# import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from cv_bridge import CvBridge
import message_filters
# import ROS2 message libraries
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from nav_msgs.msg import Odometry
# import custom message libraries
from driverless_msgs.msg import Cone, ConeDetectionStamped
from fs_msgs.msg import ControlCommand

# other python modules
from math import sqrt, atan2, pi, sin, cos, atan
import cv2
import numpy as np
import scipy.interpolate as scipy_interpolate # for spline calcs
from typing import Tuple, List, Optional
import time
import sys
import os
import getopt
import logging
import datetime
import pathlib

from transforms3d.euler import quat2euler

# import required sub modules
from driverless_common.point import Point

# initialise logger
LOGGER = logging.getLogger(__name__)

# translate ROS image messages to OpenCV
cv_bridge = CvBridge()

# image display geometry
SCALE = 20
WIDTH = 20*SCALE # 10m either side
HEIGHT = 20*SCALE # 20m forward
ORIGIN = Point(0, 0)
IMG_ORIGIN = Point(int(WIDTH/2), HEIGHT)

# display colour constants
Colour = Tuple[int, int, int]
YELLOW_DISP_COLOUR: Colour = (0, 255, 255) # bgr - yellow
BLUE_DISP_COLOUR: Colour = (255, 0, 0) # bgr - blue
ORANGE_DISP_COLOUR: Colour = (0, 165, 255) # bgr - orange

LEFT_CONE_COLOUR = Cone.BLUE
RIGHT_CONE_COLOUR = Cone.YELLOW


def robot_pt_to_img_pt(x: float, y: float) -> Point:
    """
    Converts a relative depth from the camera into image coords
    * param x: x coord
    * param y: y coord
    * return: Point int pixel coords
    """
    return Point(
        int(round(WIDTH/2 - y*SCALE)),
        int(round(HEIGHT - x*SCALE)),
    )


def dist(a: Point, b: Point) -> float:
    return sqrt((a.x-b.x)**2 + (a.y-b.y)**2)


def cone_to_point(cone: Cone) -> Point:
    return Point(
        cone.location.x,
        cone.location.y,
    )


def approximate_b_spline_path(
    x: list, 
    y: list, 
    n_path_points: int,
    degree: int = 3
) -> Tuple[list, list]:
    """
    ADAPTED FROM: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/BSplinePath/bspline_path.py \n
    Approximate points with a B-Spline path
    * param x: x position list of approximated points
    * param y: y position list of approximated points
    * param n_path_points: number of path points
    * param degree: (Optional) B Spline curve degree
    * return: x and y position list of the result path
    """

    t: int = range(len(x))
    # interpolate for the length of the input cone list
    x_list = list(scipy_interpolate.splrep(t, x, k=degree))
    y_list = list(scipy_interpolate.splrep(t, y, k=degree))

    # add 4 'zero' components to align matrices
    x_list[1] = x + [0.0, 0.0, 0.0, 0.0]
    y_list[1] = y + [0.0, 0.0, 0.0, 0.0]

    ipl_t = np.linspace(0.0, len(x) - 1, n_path_points)
    spline_x = scipy_interpolate.splev(ipl_t, x_list)
    spline_y = scipy_interpolate.splev(ipl_t, y_list)

    return spline_x, spline_y


def midpoint(p1: list, p2: list):
    """
    Retrieve midpoint between two points 
    * param p1: [x,y] coords of point 1
    * param p2: [x,y] coords of point 2
    * return: x,y tuple of midpoint coord
    """
    return (p1[0]+p2[0])/2, (p1[1]+p2[1])/2


def marker_msg(
    x_coord: float, 
    y_coord: float, 
    ID: int, 
    header: Header,
) -> Marker: 
    """
    Creates a Marker object for cones or a car.
    * param x_coord: x position relative to parent frame
    * param y_coord: y position relative to parent frame
    * param ID: Unique for markers in the same frame
    * param header: passed in because creating time is dumb
    * return: Marker
    """

    marker = Marker()
    marker.header = header
    marker.ns = "current_path"
    marker.id = ID
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.pose.position.x = x_coord
    marker.pose.position.y = y_coord
    marker.pose.position.z = 0.0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # scale out of 1x1x1m
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    marker.color.a = 1.0 # alpha
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    marker.lifetime = Duration(sec=1, nanosec=0)

    return marker


class LocalSpline(Node):
    def __init__(self, spline_len: int):
        super().__init__("local_spline")

        # subscribers
        cones_sub = message_filters.Subscriber(
            self, ConeDetectionStamped, "/detector/cone_detection"
        )
        odom_sub = message_filters.Subscriber(
            self, Odometry, "/testing_only/odom"
        )
        synchronizer = message_filters.TimeSynchronizer(
            fs=[cones_sub, odom_sub],
            queue_size=30,
        )
        synchronizer.registerCallback(self.callback)

        # publishers
        self.path_img_publisher: Publisher = self.create_publisher(Image, "/local_spline/path_img", 1)
        self.path_marker_publisher: Publisher = self.create_publisher(MarkerArray, "/local_spline/path_marker_array", 1)
        self.control_publisher: Publisher = self.create_publisher(ControlCommand, "/control_command", 10)

        LOGGER.info("---Local Spline Node Initalised---")

        self.spline_len = spline_len


    def callback(self, cone_msg: ConeDetectionStamped, odom_msg: Odometry):
        LOGGER.info("Received detection")

        cones: List[Cone] = cone_msg.cones
        # create black image
        debug_img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)


        ## PROCESS DETECTED CONES
        yellow_list: List[float] = []
        blue_list: List[float] = []
        for cone in cones:
            if cone.color == Cone.YELLOW:
                colour = YELLOW_DISP_COLOUR
                yellow_list.append([cone.location.x, cone.location.y])
            elif cone.color == Cone.BLUE:
                colour = BLUE_DISP_COLOUR
                blue_list.append([cone.location.x, cone.location.y])
            else:
                colour = (255, 255, 255)
            
            # draws location of cone w/ colour
            cv2.drawMarker(
                debug_img, 
                robot_pt_to_img_pt(cone.location.x, cone.location.y).to_tuple(),
                colour,
                markerType=cv2.MARKER_SQUARE,
                markerSize=5,
                thickness=5
            )


        ## TARGET SPLINE PLANNER
        tx: List[float] = []
        ty: List[float] = []

        # can't interpolate with less than 2 points
        if len(yellow_list) > 1 and len(blue_list) > 1:
            yellow_x: List[float] = []
            yellow_y: List[float] = []
            blue_x: List[float] = []
            blue_y: List[float] = []

            # indice degree of interpolation determined by num cones
            y_degree = len(yellow_list)-1
            b_degree = len(blue_list)-1
            # cubic (3rd degree) maximum
            if y_degree > 3: y_degree = 3
            if b_degree > 3: b_degree = 3

            # sort by closest cones to join waypoints
            yellow_sort = sorted(yellow_list, key=lambda x: (x[0],x[1]))
            blue_sort = sorted(blue_list, key=lambda x: (x[0],x[1]))
            for i in yellow_sort: # each cone coord
                # ref frame so x is forward from the car (this is graph axis y)
                yellow_x.append(-i[1]) # negative because we mixed up yellow/blue track sides
                yellow_y.append(i[0])
            for i in blue_sort:
                blue_x.append(-i[1])
                blue_y.append(i[0])
            # retrieves spline lists (x,y)
            yx, yy = approximate_b_spline_path(yellow_x, yellow_y, self.spline_len, degree=y_degree)
            bx, by = approximate_b_spline_path(blue_x, blue_y, self.spline_len, degree=b_degree)

            # find midpoint between splines at each point to make target path
            for i in range(self.spline_len):
                mid_x, mid_y = midpoint([yx[i], yy[i]], [bx[i], by[i]])
                tx.append(mid_x)
                ty.append(mid_y)


        ## ORIGINAL BANG-BANG CODE
        left_cones = [c for c in cones if c.color == LEFT_CONE_COLOUR]
        right_cones = [c for c in cones if c.color == RIGHT_CONE_COLOUR]

        closest_left: Optional[Cone] = None
        closest_right: Optional[Cone] = None
        if len(left_cones) > 0:
            closest_left = min(left_cones, key=lambda c: dist(ORIGIN, cone_to_point(c)))
        if len(right_cones) > 0:
            closest_right = min(right_cones, key=lambda c: dist(ORIGIN, cone_to_point(c)))
        
        # if we have two cones, check if they are greater than 5 meters apart
        if closest_left is not None and closest_right is not None:
            if dist(cone_to_point(closest_left), cone_to_point(closest_right)) > 5:
                # if so - remove the furthest cone from the targeting
                left_dist = dist(ORIGIN, cone_to_point(closest_left))
                right_dist = dist(ORIGIN, cone_to_point(closest_right))
                if left_dist <= right_dist:
                    closest_right = None
                else:
                    closest_left = None

        target: Optional[Point] = None
        if closest_left is not None and closest_right is not None:
            target = Point(
                x=closest_left.location.x + (closest_right.location.x - closest_left.location.x)/2,
                y=closest_left.location.y + (closest_right.location.y - closest_left.location.y)/2,
            )
        elif closest_left is not None:
            target = Point(
                x=closest_left.location.x,
                y=closest_left.location.y - 2,
            )
        elif closest_right is not None:
            target = Point(
                x=closest_right.location.x,
                y=closest_right.location.y + 2,
            )
                

        # overwrite target if there was a spline target path
        # uses the 2 closest method if not
        if tx != []:
            target_index = round(self.spline_len / 3) # 1/3 along
            target = Point(ty[target_index], -tx[target_index]) 

            # spline visualisation
            path_markers: List[Marker] = []
            for t in range(len(tx)):
                # draw each element in target spline
                cv2.drawMarker(
                    debug_img, 
                    robot_pt_to_img_pt(ty[t], -tx[t]).to_tuple(),
                    (0, 0, 255),
                    markerType=cv2.MARKER_SQUARE,
                    markerSize=1,
                    thickness=2
                )

                # target spline markers for rviz
                x = odom_msg.pose.pose.position.x
                y = odom_msg.pose.pose.position.y
                w = odom_msg.pose.pose.orientation.w
                i = odom_msg.pose.pose.orientation.x
                j = odom_msg.pose.pose.orientation.y
                k = odom_msg.pose.pose.orientation.z
                # i, j, k angles in rad
                ai, aj, ak = quat2euler([w, i, j, k])
                # displacement from car to target element
                x_dist = x + (tx[t]*sin(ak) + ty[t]*cos(ak))
                y_dist = y + (ty[t]*sin(ak) - tx[t]*cos(ak))
                # add on each cone to published array
                path_markers.append(marker_msg(
                    x_dist,
                    y_dist,
                    t, 
                    odom_msg.header,
                ))

            # create message for all cones on the track
            path_markers_msg = MarkerArray(markers=path_markers)
            self.path_publisher.publish(path_markers_msg) # publish marker points data


        ## APPROACH TARGET
        if target is not None:
            # velocity control
            # init constants
            Kp_vel: float = 2
            vel_max: float = 8
            vel_min = vel_max/2
            throttle_max: float = 0.3 # m/s^2

            # get car vel
            vel_x: float = odom_msg.twist.twist.linear.x
            vel_y: float = odom_msg.twist.twist.linear.y
            vel: float = sqrt(vel_x**2 + vel_y**2)
            
            # target velocity proportional to angle
            target_vel: float = vel_max - (abs(atan(target.y / target.x))) * Kp_vel
            if target_vel < vel_min: target_vel = vel_min
            LOGGER.info(f"Target vel: {target_vel}")

            # increase proportionally as it approaches target
            throttle_scalar: float = (1 - (vel / target_vel)) 
            if throttle_scalar > 0: calc_throttle = throttle_max * throttle_scalar
            # if its over maximum, cut throttle
            elif throttle_scalar <= 0: calc_throttle = 0

            # steering control
            Kp_ang: float = 1.4
            ang_max: float = 7.0

            steering_angle = -((pi/2) - atan2(target.x, target.y))*5
            LOGGER.info(f"Target angle: {steering_angle}")
            calc_steering = Kp_ang * steering_angle / ang_max

            # publish message
            control_msg = ControlCommand()
            control_msg.throttle = float(calc_throttle)
            control_msg.steering = float(calc_steering)
            control_msg.brake = 0.0

            self.control_publisher.publish(control_msg)

            # draw target
            target_img_pt = robot_pt_to_img_pt(target.x, target.y)
            cv2.drawMarker(
                debug_img, 
                robot_pt_to_img_pt(target.x, target.y).to_tuple(),
                (0, 0, 255),
                markerType=cv2.MARKER_TILTED_CROSS,
                markerSize=10,
                thickness=2
            )
            target_img_angle = atan2(target_img_pt.y - IMG_ORIGIN.y, target_img_pt.x - IMG_ORIGIN.x)
            # draw angle line
            cv2.line(
                debug_img,
                (int(50*cos(target_img_angle) + IMG_ORIGIN.x), int(50*sin(target_img_angle) + IMG_ORIGIN.y)),
                IMG_ORIGIN.to_tuple(),
                (0, 0, 255)
            )
            # add text for targets data
            cv2.putText(
                debug_img, "Targets", (10, HEIGHT-40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2
            )
            text_angle = "Steering: "+str(round(steering_angle, 2))
            cv2.putText(
                debug_img, text_angle, (10, HEIGHT-25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2
            )
            text_vel = "Velocity: "+str(round(target_vel, 2))
            cv2.putText(
                debug_img, text_vel, (10, HEIGHT-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2
            )

        self.debug_img_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))



def main(args=sys.argv[1:]):
    # defaults args
    loglevel = 'info'
    print_logs = False
    spline_len = 200

    # processing args
    opts, arg = getopt.getopt(args, str(), ['log=', 'print_logs', 'length='])

    # TODO: provide documentation for different options
    for opt, arg in opts:
        if opt == '--log':
            loglevel = arg
        elif opt == '--print_logs':
            print_logs = True
        elif opt == '--length':
            spline_len = arg

    # validating args
    numeric_level = getattr(logging, loglevel.upper(), None)

    if not isinstance(numeric_level, int):
        raise ValueError('Invalid log level: %s' % loglevel)

    if not isinstance(spline_len, int):
        raise ValueError('Invalid range: %s. Must be int' % spline_len)

    # setting up logging
    path = str(pathlib.Path(__file__).parent.resolve())
    if not os.path.isdir(path + '/logs'):
        os.mkdir(path + '/logs')

    date = datetime.datetime.now().strftime('%d_%m_%Y_%H_%M_%S')
    logging.basicConfig(
        filename=f'{path}/logs/{date}.log',
        filemode='w',
        format='%(asctime)s | %(levelname)s:%(name)s: %(message)s',
        datefmt='%I:%M:%S %p',
        # encoding='utf-8',
        level=numeric_level,
    )

    # terminal stream
    if print_logs:
        stdout_handler = logging.StreamHandler(sys.stdout)
        LOGGER.addHandler(stdout_handler)

    LOGGER.info(f'args = {args}')

    # begin ros node
    rclpy.init(args=args)

    node = LocalSpline(spline_len)
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])

