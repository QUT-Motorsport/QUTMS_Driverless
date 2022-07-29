"""
This is highly likely going to be unused
- it's good for getting perfect 'detections' from the sim
but not much else
"""

from math import atan, atan2, cos, pi, sin, sqrt

import numpy as np
import scipy.interpolate as scipy_interpolate
from transforms3d.euler import quat2euler

import message_filters
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDrive
from builtin_interfaces.msg import Duration
from driverless_msgs.msg import Cone, ConeDetectionStamped
from geometry_msgs.msg import Point as ROSPoint
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from visualization_msgs.msg import Marker

from driverless_common.point import Point

from typing import List, Optional, Tuple

ORIGIN = Point(0, 0)

LEFT_CONE_COLOUR = Cone.BLUE
RIGHT_CONE_COLOUR = Cone.YELLOW


def dist(a: Point, b: Point) -> float:
    return sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


def cone_to_point(cone: Cone) -> Point:
    return Point(
        cone.location.x,
        cone.location.y,
    )


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


def target_line_mkr(pose_msg, path_markers):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.ns = "current_path"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD

    marker.pose.position = pose_msg.pose.pose.position
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    # scale out of 1x1x1m
    marker.scale.x = 0.2
    marker.scale.y = 0.0
    marker.scale.z = 0.0

    marker.points = path_markers

    marker.color.a = 1.0  # alpha
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    marker.lifetime = Duration(sec=1, nanosec=0)
    return marker


class LocalPursuit(Node):
    # init constants
    spline_len: int = 200
    Kp_ang: float = 5
    Kp_vel: float = 2
    vel_max: float = 5
    vel_min = vel_max / 2
    throttle_max: float = 0.3  # m/s^2

    def __init__(self):
        super().__init__("local_pursuit")

        # sync subscribers
        pose_sub = message_filters.Subscriber(self, PoseWithCovarianceStamped, "/zed2i/zed_node/pose_with_covariance")
        vel_sub = message_filters.Subscriber(self, TwistWithCovarianceStamped, "/imu/velocity")
        detection_sub = message_filters.Subscriber(self, ConeDetectionStamped, "/vision/cone_detection")
        synchronizer = message_filters.ApproximateTimeSynchronizer(
            fs=[pose_sub, vel_sub, detection_sub], queue_size=30, slop=0.2
        )
        synchronizer.registerCallback(self.callback)

        # publishers
        self.path_marker_publisher: Publisher = self.create_publisher(Marker, "/local_spline/path_marker", 1)
        self.control_publisher: Publisher = self.create_publisher(AckermannDrive, "/driving_command", 10)

        self.get_logger().info("---Local Pursuit Node Initalised---")

    def callback(
        self, pose_msg: PoseWithCovarianceStamped, vel_msg: TwistWithCovarianceStamped, cone_msg: ConeDetectionStamped
    ):
        self.get_logger().debug("Received detection")

        # safety critical, set to 0 if not good detection
        control_msg = AckermannDrive()
        control_msg.steering_angle = 0.0
        control_msg.acceleration = 0.0
        control_msg.jerk = 1.0

        cones: List[Cone] = cone_msg.cones

        ## PROCESS DETECTED CONES
        yellow_list: List[float] = []
        blue_list: List[float] = []
        for cone in cones:
            if cone.color == Cone.YELLOW:
                yellow_list.append([cone.location.x, cone.location.y])
            elif cone.color == Cone.BLUE:
                blue_list.append([cone.location.x, cone.location.y])

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
            if y_degree > 3:
                y_degree = 3
            if b_degree > 3:
                b_degree = 3

            # sort by closest cones to join waypoints
            yellow_sort = sorted(yellow_list, key=lambda x: (x[0], x[1]))
            blue_sort = sorted(blue_list, key=lambda x: (x[0], x[1]))
            for i in yellow_sort:  # each cone coord
                # ref frame so x is forward from the car (this is graph axis y)
                yellow_x.append(-i[1])  # negative because we mixed up yellow/blue track sides
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

            target_index = round(self.spline_len / 5)  # 1/5th along
            target = Point(ty[target_index], -tx[target_index])

            # spline visualisation
            path_markers: List[Marker] = []
            for t in range(len(tx)):
                # target spline markers for rviz
                # i, j, k angles in rad
                ai, aj, ak = quat2euler(
                    [
                        pose_msg.pose.pose.orientation.w,
                        pose_msg.pose.pose.orientation.x,
                        pose_msg.pose.pose.orientation.y,
                        pose_msg.pose.pose.orientation.z,
                    ]
                )
                # displacement from car to target element
                x_dist = tx[t] * sin(ak) + ty[t] * cos(ak)
                y_dist = ty[t] * sin(ak) - tx[t] * cos(ak)

                line_point = ROSPoint()
                line_point.x = x_dist
                line_point.y = y_dist
                line_point.z = 0.0
                path_markers.append(line_point)
            # create message for all cones on the track
            self.path_marker_publisher.publish(target_line_mkr(pose_msg, path_markers))  # publish marker points data

        ## ORIGINAL BANG-BANG CODE
        else:
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
                    x=closest_left.location.x + (closest_right.location.x - closest_left.location.x) / 2,
                    y=closest_left.location.y + (closest_right.location.y - closest_left.location.y) / 2,
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

            # publish message
            control_msg.steering_angle = steering_angle
            control_msg.acceleration = calc_throttle
            control_msg.jerk = 0.0

        self.control_publisher.publish(control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalPursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
