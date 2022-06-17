# import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from cv_bridge import CvBridge
# import ROS2 message libraries
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
# import custom message libraries
from driverless_msgs.msg import Cone, ConeDetectionStamped
from fs_msgs.msg import Track

# other python modules
from math import hypot, atan2, pi, sin, cos
import cv2
import numpy as np
from typing import Tuple, List, Optional
import time
from sklearn.neighbors import KDTree
from transforms3d.euler import quat2euler
import matplotlib.pyplot as plt

# import required sub modules
from driverless_common.point import Point
from .map_cone import MapCone

# translate ROS image messages to OpenCV
cv_bridge = CvBridge()

# image display geometry
SCALE = 4
HEIGHT = (10+170)*SCALE
WIDTH = (50+50)*SCALE

def coord_to_img(x: float, y: float) -> Point:
    """
    Converts a relative depth from the camera into image coords
    * param x: x coord
    * param y: y coord
    * return: Point int pixel coords
    """
    return Point(
        int(round(50*SCALE + x*SCALE)),
        int(round(170*SCALE - y*SCALE)),
    )


def wrap_to_pi(angle: float) -> float: # in rads
    return (angle + np.pi) % (2 * np.pi) - np.pi


def predict(odom_msg: Odometry, R: np.ndarray) -> Tuple[np.ndarray]:
    """ Covariance from odom
    xx, xy, xz, xi, xj, xk
    yx, yy, yz, yi, yj, yk
    zx, zy, zz, zi, zj, zk
    ix, iy, iz, ii, ij, ik
    jx, jy, jz, ji, jj, jk
    kx, ky, kz, ki, kj, kk
    """

    # i, j, k angles in rad
    ai, aj, ak = quat2euler([
        odom_msg.pose.pose.orientation.w,
        odom_msg.pose.pose.orientation.x,
        odom_msg.pose.pose.orientation.y,
        odom_msg.pose.pose.orientation.z,
    ])

    x = odom_msg.pose.pose.position.x
    y = odom_msg.pose.pose.position.y
    theta = ak

    muR = np.array([x, y, theta]) # robot mean

    cov = odom_msg.pose.covariance
    # cov = np.reshape(cov, (6,6)) # turns out this is a matrix of zeros from sim. 
    # but IRL we will have covariance from the SBG
    cov = np.diag(np.random.rand(6)*0.01) # so mke some noise
    SigmaR = np.array([[cov[0,0], cov[0,1], cov[0,5]],
                      [cov[1,0], cov[1,1], cov[1,5]],
                      [cov[5,0], cov[5,1], cov[5,5]]])
    
    return muR, SigmaR


def init_landmark(cone: Tuple[float, float], # (range,bearing)
                  Q: np.ndarray, 
                  mu: np.ndarray, 
                  Sigma: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        
    new_x = mu[0] + cone[0]*cos(mu[2]+cone[1]) # add local xy to robot xy
    new_y = mu[1] + cone[0]*sin(mu[2]+cone[1])
    mu = np.append(mu, [new_x, new_y]) # append new landmark

    # landmark Jacobian
    Lz = np.array([[cos(mu[2]+cone[1]), cone[0]*-sin(mu[2]+cone[1])],
                   [sin(mu[2]+cone[1]), cone[0]*cos(mu[2]+cone[1])]])

    sig_len = len(Sigma)
    new_sig = np.zeros((sig_len+2, sig_len+2)) # create zeros
    new_sig[0:sig_len, 0:sig_len] = Sigma # top left corner is existing sigma
    new_sig[sig_len:sig_len+2, sig_len:sig_len+2] = Lz @ Q @ Lz.T # bottom right is new lm

    return mu, new_sig # new sigma overwrites sigma


def update(track: np.ndarray,
           index: int, 
           cone: Tuple[float, float], # (range,bearing)
           Q: np.ndarray,
           mu: np.ndarray, 
           Sigma: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:

    i = index*2+3
    muL = mu[i:i+2] # omit colour from location mean

    r = hypot(mu[0]-muL[0], mu[1]-muL[1]) # range to landmark
    b = wrap_to_pi( atan2(muL[1]-mu[1], muL[0]-mu[0]) - mu[2]) # bearing to lm
    h = [r, b]

    sig_len = len(Sigma) # length of robot+landmarks we've seen so far

    G = np.zeros((2, sig_len))
    # robot jacobian
    GR = [[ -(muL[0]-mu[0])/r, -(muL[1]-mu[1])/r, 0 ], \
          [ (muL[1]-mu[1])/(r**2), -(muL[0]-mu[0])/(r**2), 1 ] ]
    # landmark jacobian
    GL = [[ (muL[0]-mu[0])/r, (muL[1]-mu[1])/r ], \
          [ -(muL[1]-mu[1])/(r**2), (muL[0]-mu[0])/(r**2) ] ]

    G[0:2, 0:3] = GR # first 2 rows, 3 columns
    G[0:2, i:i+2] = GL # index columns, 2 rows

    K = Sigma@G.T @ np.linalg.inv(G@Sigma@G.T + Q)
    z_h = np.reshape([cone[0]-h[0], cone[1]-h[1]], (-1,1)) # turn into a 2D column array

    addon = (K@z_h).T
    mu = mu + addon[0] # because this was 2D... had to take 'first' element
    Sigma = (np.eye(sig_len) - K@G)@Sigma

    track[index, :2] = mu[i:i+2] # track is just mu without robot

    return mu, Sigma, track


class EKFSlam(Node):
    R = np.diag([0.01, 0.01, 0.01]) # very confident of odom (cause its OP)
    Q = np.diag([.96, 0.45])**2 # detections are a bit meh
    radius = 3 # nn kdtree nearch
    leaf = 50 # nodes per tree before it starts brute forcing?

    def __init__(self):
        super().__init__("ekf_slam")

        self.create_subscription(Odometry, "/testing_only/odom", self.odom_callback, 10)
        self.create_subscription(ConeDetectionStamped, "/vision/cone_detection", self.callback, 10)
        # some service call here from simple controller to send when the car has completed a lap
        # sub to track for all cone locations relative to car start point
        self.create_subscription(Track, "/testing_only/track", self.map_callback, 10)

        # publishers
        self.map_img_publisher: Publisher = self.create_publisher(Image, "/slam/map_image", 1)
        # map cone loc publisher
        self.real_map_img_publisher: Publisher = self.create_publisher(Image, "/slam/real_map_image", 1)
        self.get_logger().info("---SLAM node initialised---")

        self.odom_msg = None

        self.mu = np.array([3.0,0.0,0.0]) # initial pose
        self.Sigma = np.diag([0.01, 0.01, 0.01])
        self.track = []
        self.blue_indexes = []
        self.yellow_indexes = []

        plt.figure()

    
    def odom_callback(self, odom_msg: Odometry):
        self.get_logger().debug("Received Odom")
        self.odom_msg = odom_msg # odom updates about 20x faster than vision


    def callback(self, cone_msg: ConeDetectionStamped):
        self.get_logger().debug("Received detection")

        while self.odom_msg == None: # sometimes odom sub breaks
            time.sleep(0.5)

        # predict car location (pretty accurate odom)
        muR, SigmaR = predict(self.odom_msg, self.R)
        self.mu[0:3] = muR
        self.Sigma[0:3,0:3] = SigmaR

        # process detected cones
        cones: List[Cone] = cone_msg.cones
        for cone in cones:
            if cone.color==2: cone.color = 1 # count orange as yellow cause simplicity
            det = MapCone(cone) # detection with properties
            if det.range > 12: continue # out of range dont care
            
            mapx = self.mu[0] + det.range*cos(self.mu[2]+det.bearing)
            mapy = self.mu[1] + det.range*sin(self.mu[2]+det.bearing)

            on_map = False # by default its a new detection

            same_track = [] # start empty for this colour
            if self.track != []: 
                same_track = self.track[self.track[:, 2]==det.colour] # extract same colours

            if len(same_track) != 0: # this spline has been populated with cones
                neighbourhood = KDTree(same_track[:,:2], leaf_size=self.leaf)
                check = np.reshape([mapx,mapy], (1,-1)) # turn into a 2D row array
                ind = neighbourhood.query_radius(check, r=self.radius) # check neighbours in radius
                close = ind[0] # index from the single colour list
                if close.size != 0: 
                    on_map = True
                    # update the index of the matched cone on the track
                    if det.colour == 0: self.mu, self.Sigma, self.track = update(self.track, self.blue_indexes[close[0]], det.sense_rb, self.Q, self.mu, self.Sigma)
                    elif det.colour == 1: self.mu, self.Sigma, self.track = update(self.track, self.yellow_indexes[close[0]], det.sense_rb, self.Q, self.mu, self.Sigma)

            if not on_map:
                if self.track == []: # first in this list
                    self.track = np.array([mapx, mapy, det.colour])
                    self.track = np.reshape(self.track, (1,-1)) # turn 2D
                else: # otherwise append vertically
                    self.track = np.vstack([self.track, [mapx, mapy, det.colour]])
                # get index relative to the whole track 
                if det.colour == 0: self.blue_indexes.append(len(self.track)-1)
                elif det.colour == 1: self.yellow_indexes.append(len(self.track)-1)
                # initialise new landmark
                self.mu, self.Sigma = init_landmark(det.sense_rb, self.Q, self.mu, self.Sigma)

        blue_track = self.track[self.track[:, 2]==0]
        yellow_track = self.track[self.track[:, 2]==1]
        # Plotting
        map_img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        cv2.drawMarker(
            map_img, 
            coord_to_img(muR[0],muR[1]).to_tuple(),
            (255, 255, 255),
            markerType=cv2.MARKER_SQUARE,
            markerSize=4,
            thickness=2
        )

        for cone in blue_track:
            cv2.drawMarker(
                map_img, 
                coord_to_img(cone[0],cone[1]).to_tuple(),
                (255, 0, 0),
                markerType=cv2.MARKER_TRIANGLE_UP,
                markerSize=6,
                thickness=2
            )
        for cone in yellow_track:
            cv2.drawMarker(
                map_img, 
                coord_to_img(cone[0],cone[1]).to_tuple(),
                (0, 255, 255),
                markerType=cv2.MARKER_TRIANGLE_UP,
                markerSize=6,
                thickness=2
            )

        self.map_img_publisher.publish(cv_bridge.cv2_to_imgmsg(map_img, encoding="bgr8"))


    def map_callback(self, track_msg: Track):       
        # track cone list is taken as coords relative to the initial car position
        track=track_msg.track
        map_img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)  
        for cone in track:
            if cone.color == Cone.BLUE:
                disp_col = (255, 0, 0)
            elif cone.color == Cone.YELLOW:
                disp_col = (0, 255, 255)
            elif cone.color == Cone.ORANGE_BIG:
                disp_col = (0, 100, 255)

            cv2.drawMarker(
                map_img, 
                coord_to_img(cone.location.x,cone.location.y).to_tuple(),
                disp_col,
                markerType=cv2.MARKER_TRIANGLE_UP,
                markerSize=6,
                thickness=2
            )
        self.real_map_img_publisher.publish(cv_bridge.cv2_to_imgmsg(map_img, encoding="bgr8"))


def main(args=None):
    # begin ros node
    rclpy.init(args=args)

    node = EKFSlam()
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()
