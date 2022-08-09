from math import atan2, cos, hypot, sin
import time

import cv2
import numpy as np
from sklearn.neighbors import KDTree
from transforms3d.euler import quat2euler

from cv_bridge import CvBridge
import message_filters
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from driverless_msgs.msg import Cone, ConeDetectionStamped, ConeWithCovariance, TrackDetectionStamped
from geometry_msgs.msg import Point as ROSPoint
from geometry_msgs.msg import PoseWithCovarianceStamped

from driverless_common.cone_props import ConeProps
from driverless_common.point import Point

from typing import List, Tuple

# translate ROS image messages to OpenCV
cv_bridge = CvBridge()

# image display geometry
SCALE = 4
HEIGHT = (10 + 170) * SCALE
WIDTH = (50 + 50) * SCALE


def coord_to_img(x: float, y: float) -> Point:
    """
    Converts a relative depth from the camera into image coords
    * param x: x coord
    * param y: y coord
    * return: Point int pixel coords
    """
    return Point(
        int(round(50 * SCALE + x * SCALE)),
        int(round(170 * SCALE - y * SCALE)),
    )


def wrap_to_pi(angle: float) -> float:  # in rads
    return (angle + np.pi) % (2 * np.pi) - np.pi


def predict(pose_msg: PoseWithCovarianceStamped, R: np.ndarray) -> Tuple[np.ndarray]:
    """Covariance from odom
    xx, xy, xz, xi, xj, xk
    yx, yy, yz, yi, yj, yk
    zx, zy, zz, zi, zj, zk
    ix, iy, iz, ii, ij, ik
    jx, jy, jz, ji, jj, jk
    kx, ky, kz, ki, kj, kk
    """

    # i, j, k angles in rad
    ai, aj, ak = quat2euler(
        [
            pose_msg.pose.pose.orientation.w,
            pose_msg.pose.pose.orientation.x,
            pose_msg.pose.pose.orientation.y,
            pose_msg.pose.pose.orientation.z,
        ]
    )

    x = pose_msg.pose.pose.position.x
    y = pose_msg.pose.pose.position.y
    theta = ak

    muR = np.array([x, y, theta])  # robot mean

    cov = pose_msg.pose.covariance
    cov = np.reshape(cov, (6, 6))
    SigmaR = np.array(
        [[cov[0, 0], cov[0, 1], cov[0, 5]], [cov[1, 0], cov[1, 1], cov[1, 5]], [cov[5, 0], cov[5, 1], cov[5, 5]]]
    )

    return muR, SigmaR


def init_landmark(
    cone: Tuple[float, float], Q: np.ndarray, mu: np.ndarray, Sigma: np.ndarray  # (range,bearing)
) -> Tuple[np.ndarray, np.ndarray]:

    new_x = mu[0] + cone[0] * cos(mu[2] + cone[1])  # add local xy to robot xy
    new_y = mu[1] + cone[0] * sin(mu[2] + cone[1])
    mu = np.append(mu, [new_x, new_y])  # append new landmark

    # landmark Jacobian
    Lz = np.array(
        [
            [cos(mu[2] + cone[1]), cone[0] * -sin(mu[2] + cone[1])],
            [sin(mu[2] + cone[1]), cone[0] * cos(mu[2] + cone[1])],
        ]
    )

    sig_len = len(Sigma)
    new_sig = np.zeros((sig_len + 2, sig_len + 2))  # create zeros
    new_sig[0:sig_len, 0:sig_len] = Sigma  # top left corner is existing sigma
    new_sig[sig_len : sig_len + 2, sig_len : sig_len + 2] = Lz @ Q @ Lz.T  # bottom right is new lm

    return mu, new_sig  # new sigma overwrites sigma


def update(
    track: np.ndarray,
    index: int,
    cone: Tuple[float, float],  # (range,bearing)
    Q: np.ndarray,
    mu: np.ndarray,
    Sigma: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:

    i = index * 2 + 3
    muL = mu[i : i + 2]  # omit colour from location mean

    r = hypot(mu[0] - muL[0], mu[1] - muL[1])  # range to landmark
    b = wrap_to_pi(atan2(muL[1] - mu[1], muL[0] - mu[0]) - mu[2])  # bearing to lm
    h = [r, b]

    sig_len = len(Sigma)  # length of robot+landmarks we've seen so far

    G = np.zeros((2, sig_len))
    # robot jacobian
    GR = [
        [-(muL[0] - mu[0]) / r, -(muL[1] - mu[1]) / r, 0],
        [(muL[1] - mu[1]) / (r**2), -(muL[0] - mu[0]) / (r**2), 1],
    ]
    # landmark jacobian
    GL = [[(muL[0] - mu[0]) / r, (muL[1] - mu[1]) / r], [-(muL[1] - mu[1]) / (r**2), (muL[0] - mu[0]) / (r**2)]]

    G[0:2, 0:3] = GR  # first 2 rows, 3 columns
    G[0:2, i : i + 2] = GL  # index columns, 2 rows

    K = Sigma @ G.T @ np.linalg.inv(G @ Sigma @ G.T + Q)
    z_h = np.reshape([cone[0] - h[0], cone[1] - h[1]], (-1, 1))  # turn into a 2D column array

    addon = (K @ z_h).T
    mu = mu + addon[0]  # because this was 2D... had to take 'first' element
    Sigma = (np.eye(sig_len) - K @ G) @ Sigma

    track[index, :2] = mu[i : i + 2]  # track is just mu without robot

    return mu, Sigma, track


class EKFSlam(Node):
    R = np.diag([0.05, 0.05, 0.05])  # very confident of odom (cause its OP)
    Q = np.diag([1, 0.8]) ** 2  # detections are a bit meh
    radius = 3  # nn kdtree nearch
    leaf = 30  # nodes per tree before it starts brute forcing?

    mu = np.array([3.0, 0.0, 0.0])  # initial pose
    Sigma: np.ndarray = np.diag([0.01, 0.01, 0.01])
    track: np.ndarray = []
    blue_indexes: List[int] = []
    yellow_indexes: List[int] = []
    orange_indexes: List[int] = []

    def __init__(self):
        super().__init__("ekf_slam")

        # sync subscribers
        pose_sub = message_filters.Subscriber(self, PoseWithCovarianceStamped, "/zed2i/zed_node/pose_with_covariance")
        detection_sub = message_filters.Subscriber(self, ConeDetectionStamped, "/vision/cone_detection")
        synchronizer = message_filters.ApproximateTimeSynchronizer(
            fs=[pose_sub, detection_sub], queue_size=20, slop=0.2
        )
        synchronizer.registerCallback(self.callback)

        # slam publisher
        self.slam_publisher: Publisher = self.create_publisher(TrackDetectionStamped, "/slam/track", 1)

        self.get_logger().info("---SLAM node initialised---")

    def callback(self, pose_msg: PoseWithCovarianceStamped, detection_msg: ConeDetectionStamped):
        self.get_logger().debug("Received detection")

        # predict car location (pretty accurate odom)
        muR, SigmaR = predict(pose_msg, self.R)
        self.mu[0:3] = muR
        self.Sigma[0:3, 0:3] = SigmaR

        # process detected cones
        cones: List[Cone] = detection_msg.cones
        for cone in cones:
            det = ConeProps(cone)  # detection with properties
            if det.range > 12:
                continue  # out of range dont care

            mapx = self.mu[0] + det.range * cos(self.mu[2] + det.bearing)
            mapy = self.mu[1] + det.range * sin(self.mu[2] + det.bearing)

            on_map = False  # by default its a new detection

            same_track = []  # start empty for this colour
            if self.track != []:
                same_track: np.ndarray = self.track[self.track[:, 2] == det.colour]  # extract same colours

            if len(same_track) != 0:  # this spline has been populated with cones
                neighbourhood = KDTree(same_track[:, :2], leaf_size=self.leaf)
                check = np.reshape([mapx, mapy], (1, -1))  # turn into a 2D row array
                ind = neighbourhood.query_radius(check, r=self.radius)  # check neighbours in radius
                close = ind[0]  # index from the single colour list
                if close.size != 0:
                    on_map = True
                    # update the index of the matched cone on the track
                    if det.colour == 0:
                        self.mu, self.Sigma, self.track = update(
                            self.track, self.blue_indexes[close[0]], det.sense_rb, self.Q, self.mu, self.Sigma
                        )
                    elif det.colour == 1:
                        self.mu, self.Sigma, self.track = update(
                            self.track, self.yellow_indexes[close[0]], det.sense_rb, self.Q, self.mu, self.Sigma
                        )
                    elif det.colour == 2:
                        self.mu, self.Sigma, self.track = update(
                            self.track, self.orange_indexes[close[0]], det.sense_rb, self.Q, self.mu, self.Sigma
                        )

            if not on_map:
                if self.track == []:  # first in this list
                    self.track = np.array([mapx, mapy, det.colour])
                    self.track = np.reshape(self.track, (1, -1))  # turn 2D
                else:  # otherwise append vertically
                    self.track = np.vstack([self.track, [mapx, mapy, det.colour]])
                # get i relative to the whole track
                if det.colour == 0:
                    self.blue_indexes.append(len(self.track) - 1)
                elif det.colour == 1:
                    self.yellow_indexes.append(len(self.track) - 1)
                elif det.colour == 2:
                    self.orange_indexes.append(len(self.track) - 1)
                # initialise new landmark
                self.mu, self.Sigma = init_landmark(det.sense_rb, self.Q, self.mu, self.Sigma)

        # TODO: Ordering track boundary cone lines from start to finish

        # publish track msg
        track_msg = TrackDetectionStamped()
        track_msg.header.stamp = pose_msg.header.stamp
        track_msg.header.frame_id = "map"
        for i, cone in enumerate(self.track):
            cov_i = i * 2 + 3

            curr_cov: np.ndarray = self.Sigma[cov_i : cov_i + 2, cov_i : cov_i + 2]  # 2x2 covariance to plot

            cone_msg = Cone(location=ROSPoint(x=cone[0], y=cone[1], z=0.0), color=int(cone[2]))
            cone_cov = curr_cov.flatten().tolist()

            track_msg.cones.append(ConeWithCovariance(cone=cone_msg, covariance=cone_cov))

        self.slam_publisher.publish(track_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EKFSlam()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
