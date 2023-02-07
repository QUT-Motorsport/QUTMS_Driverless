from math import atan2, pi

import numpy as np
import scipy.interpolate as scipy_interpolate

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from driverless_msgs.msg import ConeWithCovariance, PathPoint, PathStamped, TrackDetectionStamped
from eufs_msgs.msg import ConeArrayWithCovariance
from fs_msgs.msg import Cone, Track

from typing import List, Tuple

# for colour gradient based on intensity
MAX_ANGLE = 0.15


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


def midpoint(p1: List[float], p2: List[float]) -> Tuple[float]:
    """
    Retrieve midpoint between two points
    * param p1: [x,y] coords of point 1
    * param p2: [x,y] coords of point 2
    * return: x,y tuple of midpoint coord
    """
    return (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2


def angle(p1: List[float], p2: List[float]) -> float:
    """
    Retrieve angle between two points
    * param p1: [x,y] coords of point 1
    * param p2: [x,y] coords of point 2
    * return: angle in rads
    """
    x_disp = p2[0] - p1[0]
    y_disp = p2[1] - p1[1]
    return atan2(y_disp, x_disp)


def cone_array_with_covariance_to_cone_list(eufs_cones: ConeArrayWithCovariance) -> List[Cone]:
    internal_cones: List[Cone] = []
    for eufs_cone in eufs_cones.blue_cones:
        internal_cone = Cone()
        internal_cone.location = eufs_cone.point
        internal_cone.color = Cone.BLUE
        internal_cones.append(internal_cone)

    for eufs_cone in eufs_cones.yellow_cones:
        internal_cone = Cone()
        internal_cone.location = eufs_cone.point
        internal_cone.color = Cone.YELLOW
        internal_cones.append(internal_cone)

    for eufs_cone in eufs_cones.orange_cones:
        internal_cone = Cone()
        internal_cone.location = eufs_cone.point
        internal_cone.color = Cone.ORANGE_SMALL
        internal_cones.append(internal_cone)

    for eufs_cone in eufs_cones.big_orange_cones:
        internal_cone = Cone()
        internal_cone.location = eufs_cone.point
        internal_cone.color = Cone.ORANGE_BIG
        internal_cones.append(internal_cone)

    for eufs_cone in eufs_cones.unknown_color_cones:
        internal_cone = Cone()
        internal_cone.location = eufs_cone.point
        internal_cone.color = Cone.UNKNOWN
        internal_cones.append(internal_cone)

    return internal_cones


class MapPathPlanner(Node):
    spline_len: int = 3999

    def __init__(self):
        super().__init__("map_path_translator_node")

        # sub to track for all cone locations relative to car start point
        self.create_subscription(ConeArrayWithCovariance, "/ground_truth/noisy_track", self.map_callback, 10)

        # publishers
        self.track_publisher: Publisher = self.create_publisher(TrackDetectionStamped, "/sim/global_map", 1)
        self.path_publisher: Publisher = self.create_publisher(PathStamped, "/sim/path", 1)

        self.get_logger().info("---Sim Path Planner Node Initalised---")

    def map_callback(self, track_msg: ConeArrayWithCovariance):
        self.get_logger().debug("Received map")

        cones = cone_array_with_covariance_to_cone_list(track_msg)

        yellow_x: List[float] = []
        yellow_y: List[float] = []
        blue_x: List[float] = []
        blue_y: List[float] = []
        oranges: List[Cone] = []

        sim_track: List[ConeWithCovariance] = []
        for cone in cones:
            if cone.color == Cone.YELLOW:
                yellow_x.append(cone.location.x)
                yellow_y.append(cone.location.y)
                cone.color = 4
            elif cone.color == Cone.BLUE:
                blue_x.append(cone.location.x)
                blue_y.append(cone.location.y)
            elif cone.color == Cone.ORANGE_BIG:
                oranges.append(cone)

            new_cone = ConeWithCovariance()
            new_cone.cone.location = cone.location
            new_cone.cone.color = cone.color
            new_cone.covariance = [0.0, 0.0, 0.0, 0.0]
            sim_track.append(new_cone)

        if len(yellow_x) != len(yellow_y) or len(blue_x) != len(blue_y):
            self.get_logger().error("Cone coordinates must be balanced")
            return

        # 4 orange cones: 2 blue side, 2 yellow side
        for cone in oranges:
            if cone.location.x > 6:  # far pair of cones
                if cone.location.y > 0:  # blue side
                    blue_x.insert(0, cone.location.x)
                    blue_y.insert(0, cone.location.y)
                else:  # yellow side
                    yellow_x.insert(0, cone.location.x)
                    yellow_y.insert(0, cone.location.y)
            else:  # close pair of cones
                if cone.location.y > 0:  # blue side
                    blue_x.append(cone.location.x)
                    blue_y.append(cone.location.y)
                else:  # yellow side
                    yellow_x.append(cone.location.x)
                    yellow_y.append(cone.location.y)

        # retrieves spline lists (x,y)
        yx, yy = approximate_b_spline_path(yellow_x, yellow_y, self.spline_len)
        bx, by = approximate_b_spline_path(blue_x, blue_y, self.spline_len)

        tx: List[float] = []  # target spline x coords
        ty: List[float] = []  # target spline y coords
        th: List[float] = []  # target spline angles
        # find midpoint between splines at each point to make target path
        for i in range(self.spline_len):
            mid_x, mid_y = midpoint([yx[i], yy[i]], [bx[i], by[i]])
            tx.append(mid_x)
            ty.append(mid_y)
            # angle of tangent at midpoint
            th.append(angle([bx[i], by[i]], [yx[i], yy[i]]))

        VEL_ZONE = 15
        path: list[PathPoint] = []
        for i in range(0, self.spline_len - VEL_ZONE, VEL_ZONE):
            # check angle between current and 10th spline point ahead
            th_change = th[i + VEL_ZONE] - th[i]
            # keep between 360
            if th_change > pi:
                th_change = th_change - 2 * pi
            elif th_change < -pi:
                th_change = th_change + 2 * pi

            # angle relative to max angle on track
            change_pc = abs(th_change) / MAX_ANGLE * 100
            for j in range(VEL_ZONE):
                path_point = PathPoint()
                path_point.location.x = tx[i + j]
                path_point.location.y = ty[i + j]
                path_point.location.z = 0.0
                path_point.turn_intensity = change_pc
                path.append(path_point)

        # Add the first path point to the end of the list to complete the loop
        path.append(path[0])

        path_msg = PathStamped(path=path)
        path_msg.header.frame_id = "track"
        self.path_publisher.publish(path_msg)
        sim_track_msg = TrackDetectionStamped()
        sim_track_msg.header.frame_id = "track"
        sim_track_msg.cones = sim_track
        self.track_publisher.publish(sim_track_msg)


def main(args=None):
    # begin ros node
    rclpy.init(args=args)
    node = MapPathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
