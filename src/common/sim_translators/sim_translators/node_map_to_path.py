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


def sort_cones(cones, start_index=None, end_index=None):
    """
    This is a function that calculates the nearest-neighbor path between a start and end index for a list of cones.
    The cones are represented as 2D points in the form of (x, y).

    The function takes 3 parameters:
    cones: a list of the cones to be sorted.
    start_index: the index of the starting cone.
    end_index: the index of the ending cone.

    The function uses a matrix "mat" to store the Euclidean distances between each pair of cones, and uses these
    distances to calculate the nearest-neighbor path between the start and end indices.

    The path is found by starting at the start_index, finding the index of the closest cone that hasn't already
    been visited, and adding that index to the "order" list. This process is repeated until the end_index is reached.

    The final path is returned as the "order" list, which contains the indices of the cones in the order they should
    be visited.
    """
    if start_index is None:
        start_index = 0
    if end_index is None:
        end_index = len(cones) - 1

    mat = np.empty((len(cones), len(cones)))
    for i, a in enumerate(cones):
        for j, b in enumerate(cones):
            if i == j:
                continue
            mat[i][j] = np.linalg.norm([a[0] - b[0], a[1] - b[1]])

    order = [start_index]
    # Loop for each cone that needs to be ordered.
    # -2 accounts for the start and end indices that are manually handled.
    for unused in range(mat.shape[0] - 2):
        min_index = -1
        min_value = 10000
        # Get the latest ordered cone's index from the 'order' list, and loop through that cone's row in 'mat'.
        # That cone's row in 'mat' contains the distance between the cone, and every other cone on the track.
        # We can then easily find the closest unused cone and append it to the 'order' list.
        for i, dist in enumerate(mat[order[-1]]):
            if dist >= min_value or dist == 0 or i == end_index or i in order:
                continue

            min_index = i
            min_value = dist
        order.append(min_index)
    order.append(end_index)

    # Return the ordered cones.
    return [cones[order[i]] for i in range(len(order))]


class MapPathPlanner(Node):
    spline_len: int = 3999

    def __init__(self):
        super().__init__("map_path_translator_node")

        # sub to track for all cone locations relative to car start point
        self.create_subscription(ConeArrayWithCovariance, "/simulated_noise/track", self.map_callback, 10)

        # publishers
        self.track_publisher: Publisher = self.create_publisher(TrackDetectionStamped, "/sim/global_map", 1)
        self.path_publisher: Publisher = self.create_publisher(PathStamped, "/sim/path", 1)

        self.get_logger().info("---Sim Path Planner Node Initalised---")

    def map_callback(self, track_msg: ConeArrayWithCovariance):
        self.get_logger().debug("Received map")

        cones = cone_array_with_covariance_to_cone_list(track_msg)

        yellows: List[List[float]] = []
        blues: List[List[float]] = []
        oranges: List[List[float]] = []
        sim_track: List[ConeWithCovariance] = []

        for cone in cones:
            if cone.color == Cone.YELLOW:
                yellows.append([cone.location.x, cone.location.y])
            elif cone.color == Cone.BLUE:
                blues.append([cone.location.x, cone.location.y])
            elif cone.color == Cone.ORANGE_BIG:
                oranges.append([cone.location.x, cone.location.y])

            new_cone = ConeWithCovariance()
            new_cone.cone.location = cone.location
            new_cone.cone.color = cone.color
            new_cone.covariance = [0.0, 0.0, 0.0, 0.0]
            sim_track.append(new_cone)

        parsed_orange_cones = self.parse_orange_cones(oranges)
        if len(parsed_orange_cones) == 0:
            return
        blues.insert(0, parsed_orange_cones[1])
        blues.append(parsed_orange_cones[0])
        yellows.insert(0, parsed_orange_cones[3])
        yellows.append(parsed_orange_cones[2])

        # Sort the blue and yellow cones starting from the far orange cone, and ending at the close orange cone.
        ordered_blues = sort_cones(blues)
        ordered_yellows = sort_cones(yellows)

        # retrieves spline lists (x,y)
        yx, yy = approximate_b_spline_path(
            [cone[0] for cone in ordered_yellows], [cone[1] for cone in ordered_yellows], self.spline_len
        )
        bx, by = approximate_b_spline_path(
            [cone[0] for cone in ordered_blues], [cone[1] for cone in ordered_blues], self.spline_len
        )

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

    def parse_orange_cones(self, orange_cones: List[List[float]]) -> List[List[float]]:
        """
        Breaks the big orange starting cones into their position relative to the other blue/yellow cones.
        Returns format close_blue, far_blue, close_yellow, far_yellow.
        """
        if len(orange_cones) != 4:
            self.get_logger().fatal("parse_orange_cones called with less than 4 visible cones. Requires 4 cones.")
            return []

        blue_cones = [cone for cone in orange_cones if cone[1] > 0]
        yellow_cones = [cone for cone in orange_cones if cone[1] < 0]

        if blue_cones[0][0] > blue_cones[1][0]:
            blue_cones[0], blue_cones[1] = blue_cones[1], blue_cones[0]

        if yellow_cones[0][0] > yellow_cones[1][0]:
            yellow_cones[0], yellow_cones[1] = yellow_cones[1], yellow_cones[0]

        return [blue_cones[0], blue_cones[1], yellow_cones[0], yellow_cones[1]]


def main(args=None):
    # begin ros node
    rclpy.init(args=args)
    node = MapPathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
