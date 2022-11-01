"""
This is highly likely going to be unused
- it's good for getting perfect 'detections' from the sim
but not much else
"""

from math import atan, atan2, pi, sqrt

import cv2
import numpy as np
import scipy.interpolate as scipy_interpolate

from cv_bridge import CvBridge
import message_filters
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDrive
from driverless_msgs.msg import ConeWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from sensor_msgs.msg import Image

from driverless_common.draw import *
from driverless_common.point import Point, cone_to_point, dist

from typing import List, Optional, Tuple

# translate ROS image messages to OpenCV
cv_bridge = CvBridge()

ORIGIN = Point(0, 0)


def approximate_b_spline_path(x: list, y: list, n_path_points: int, degree: int = 3) -> Tuple[list, list]:
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
    return (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2


class LocalPursuit(Node):
    # init constants
    spline_len: int = 200
    Kp_ang: float = 3.5
    Kp_vel: float = 2
    vel_max: float = 5
    vel_min = vel_max / 2
    throttle_max: float = 0.3  # m/s^2

    def __init__(self):
        super().__init__("local_pursuit")

        # sync subscribers
        vel_sub = message_filters.Subscriber(self, TwistWithCovarianceStamped, "/imu/velocity")
        detection_sub = message_filters.Subscriber(self, TrackDetectionStamped, "/slam/local")
        synchronizer = message_filters.ApproximateTimeSynchronizer(
            fs=[vel_sub, detection_sub],
            queue_size=30,
            slop=0.2,
        )
        synchronizer.registerCallback(self.callback)

        # publishers
        self.control_publisher: Publisher = self.create_publisher(AckermannDrive, "/driving_command", 10)
        self.debug_img_publisher: Publisher = self.create_publisher(Image, "/debug_imgs/control_img", 1)

        self.get_logger().info("---Local Pursuit Node Initalised---")

    def callback(self, vel_msg: TwistWithCovarianceStamped, cone_msg: TrackDetectionStamped):
        self.get_logger().debug("Received detection")

        # safety critical, set to 0 if not good detection
        control_msg = AckermannDrive()
        control_msg.steering_angle = 0.0
        control_msg.acceleration = 0.0
        control_msg.jerk = 1.0

        cones: List[ConeWithCovariance] = cone_msg.cones

        # create black image
        debug_img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

        ## PROCESS DETECTED CONES
        yellow_list = [c for c in cones if c.cone.color == Cone.YELLOW]
        blue_list = [c for c in cones if c.cone.color == Cone.BLUE]
        for yellow in yellow_list:
            # draws location of cone w/ colour
            cv2.drawMarker(
                debug_img,
                loc_to_img_pt(yellow.cone.location.x, yellow.cone.location.y).to_tuple(),
                YELLOW_DISP_COLOUR,
                markerType=cv2.MARKER_SQUARE,
                markerSize=5,
                thickness=5,
            )
        for blue in blue_list:
            # draws location of cone w/ colour
            cv2.drawMarker(
                debug_img,
                loc_to_img_pt(blue.cone.location.x, blue.cone.location.y).to_tuple(),
                BLUE_DISP_COLOUR,
                markerType=cv2.MARKER_SQUARE,
                markerSize=5,
                thickness=5,
            )

        ## TARGET SPLINE PLANNER
        if len(yellow_list) > 1 and len(blue_list) > 1:  # can't interpolate with less than 2 points
            tx: List[float] = []
            ty: List[float] = []
            yellow_x: List[float] = []
            yellow_y: List[float] = []
            blue_x: List[float] = []
            blue_y: List[float] = []

            # indice degree of interpolation determined by num cones
            y_degree = len(yellow_list) - 1
            b_degree = len(blue_list) - 1
            # cubic (3rd degree) maximum
            y_degree = min(y_degree, 3)
            b_degree = min(b_degree, 3)

            # sort by closest cones to join waypoints
            yellow_sort = sorted(yellow_list, key=lambda c: (c.cone.location.x, c.cone.location.y))
            blue_sort = sorted(blue_list, key=lambda c: (c.cone.location.x, c.cone.location.y))

            for yellow in yellow_sort:  # each cone coord
                # ref frame so x is forward from the car (this is graph axis y)
                yellow_x.append(yellow.cone.location.y)  # negative because we mixed up yellow/blue track sides
                yellow_y.append(yellow.cone.location.x)
            for i in blue_sort:
                blue_x.append(blue.cone.location.y)
                blue_y.append(blue.cone.location.x)

            # retrieves spline lists (x,y)
            yx, yy = approximate_b_spline_path(yellow_x, yellow_y, self.spline_len, degree=y_degree)
            bx, by = approximate_b_spline_path(blue_x, blue_y, self.spline_len, degree=b_degree)

            # find midpoint between splines at each point to make target path
            for i in range(self.spline_len):
                mid_x, mid_y = midpoint([yx[i], yy[i]], [bx[i], by[i]])
                tx.append(mid_x)
                ty.append(mid_y)

                # draw each element in target spline
                cv2.drawMarker(
                    debug_img,
                    loc_to_img_pt(mid_y, mid_x).to_tuple(),
                    (0, 0, 255),
                    markerType=cv2.MARKER_SQUARE,
                    markerSize=1,
                    thickness=2,
                )

            target_index = round(self.spline_len / 5)  # 1/5th along
            target = Point(ty[target_index], tx[target_index])

        ## ORIGINAL BANG-BANG CODE
        else:
            closest_blue: Optional[ConeWithCovariance] = None
            closest_yellow: Optional[ConeWithCovariance] = None
            if len(blue_list) > 0:
                closest_blue = min(blue_list, key=lambda c: dist(ORIGIN, cone_to_point(c)))
            if len(yellow_list) > 0:
                closest_yellow = min(yellow_list, key=lambda c: dist(ORIGIN, cone_to_point(c)))

            # if we have two cones, check if they are greater than 5 meters apart
            if closest_blue is not None and closest_yellow is not None:
                if dist(cone_to_point(closest_blue), cone_to_point(closest_yellow)) > 5:
                    # if so - remove the furthest cone from the targeting
                    left_dist = dist(ORIGIN, cone_to_point(closest_blue))
                    right_dist = dist(ORIGIN, cone_to_point(closest_yellow))
                    if left_dist <= right_dist:
                        closest_yellow = None
                    else:
                        closest_blue = None

            target: Optional[Point] = None
            if closest_blue is not None and closest_yellow is not None:
                target = Point(
                    x=closest_blue.cone.location.x
                    + (closest_yellow.cone.location.x - closest_blue.cone.location.x) / 2,
                    y=closest_blue.cone.location.y
                    + (closest_yellow.cone.location.y - closest_blue.cone.location.y) / 2,
                )
            elif closest_blue is not None:
                target = Point(
                    x=closest_blue.cone.location.x,
                    y=closest_blue.cone.location.y - 2,
                )
            elif closest_yellow is not None:
                target = Point(
                    x=closest_yellow.cone.location.x,
                    y=closest_yellow.cone.location.y + 2,
                )

        ## APPROACH TARGET
        if target is not None:
            # velocity control
            vel = sqrt(vel_msg.twist.twist.linear.x**2 + vel_msg.twist.twist.linear.y**2)
            # target velocity proportional to angle
            target_vel: float = self.vel_max - (abs(atan(target.y / target.x))) * self.Kp_vel
            if target_vel < self.vel_min:
                target_vel = self.vel_min
            self.get_logger().debug(f"Target vel: {target_vel}")

            # increase proportionally as it approaches target
            throttle_scalar: float = 1 - (vel / target_vel)
            if throttle_scalar > 0:
                calc_throttle = self.throttle_max * throttle_scalar
            elif throttle_scalar <= 0:
                calc_throttle = 0.0  # if its over maximum, cut throttle

            # steering control
            steering_angle = -((pi / 2) - atan2(target.x, target.y)) * self.Kp_ang
            self.get_logger().debug(f"Target angle: {steering_angle}")

            target_img_pt = loc_to_img_pt(target.x, target.y)
            target_img_angle = atan2(target_img_pt.y - IMG_ORIGIN.y, target_img_pt.x - IMG_ORIGIN.x)
            # draw angle line
            cv2.line(
                debug_img,
                (int(50 * cos(target_img_angle) + IMG_ORIGIN.x), int(50 * sin(target_img_angle) + IMG_ORIGIN.y)),
                IMG_ORIGIN.to_tuple(),
                (0, 0, 255),
            )

            # publish message
            control_msg.steering_angle = steering_angle
            control_msg.acceleration = calc_throttle
            control_msg.jerk = 0.0

        self.control_publisher.publish(control_msg)
        self.debug_img_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = LocalPursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
