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


# Calculate the euclidian distance in n-space of the route r traversing cities c, ending at the path start.
PATH_DISTANCE = lambda r,c: np.sum([np.linalg.norm(c[r[p]]-c[r[p-1]]) for p in range(len(r))])
# Reverse the order of all elements from element i to element k in array r.
TWO_OPT_SWAP = lambda r,i,k: np.concatenate((r[0:i],r[k:-len(r)+i-1:-1],r[k+1:len(r)]))

def two_opt(cities,improvement_threshold): # 2-opt Algorithm adapted from https://en.wikipedia.org/wiki/2-opt
    route = np.arange(cities.shape[0]) # Make an array of row numbers corresponding to cities.
    improvement_factor = 1 # Initialize the improvement factor.
    best_distance = PATH_DISTANCE(route,cities) # Calculate the distance of the initial path.
    while improvement_factor > improvement_threshold: # If the route is still improving, keep going!
        distance_to_beat = best_distance # Record the distance at the beginning of the loop.
        for swap_first in range(1,len(route)-2): # From each city except the first and last,
            for swap_last in range(swap_first+1,len(route)): # to each of the cities following,
                new_route = TWO_OPT_SWAP(route,swap_first,swap_last) # try reversing the order of these cities
                new_distance = PATH_DISTANCE(new_route,cities) # and check the total distance with this modification.
                if new_distance < best_distance: # If the path distance is an improvement,
                    route = new_route # make this the accepted best route
                    best_distance = new_distance # and update the distance corresponding to this route.
        improvement_factor = 1 - best_distance/distance_to_beat # Calculate how much the route has improved.
    return route # When the route is no longer improving substantially, stop searching and return the route.


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

        yellows: List[Tuple[float, float]] = []
        blues: List[Tuple[float, float]] = []
        oranges: List[Tuple[float, float]] = []
        sim_track: List[ConeWithCovariance] = []

        for cone in cones:
            if cone.color == Cone.YELLOW:
                yellows.append((cone.location.x, cone.location.y))
            elif cone.color == Cone.BLUE:
                blues.append((cone.location.x, cone.location.y))
            elif cone.color == Cone.ORANGE_BIG:
                oranges.append((cone.location.x, cone.location.y))

            new_cone = ConeWithCovariance()
            new_cone.cone.location = cone.location
            new_cone.cone.color = cone.color
            new_cone.covariance = [0.0, 0.0, 0.0, 0.0]
            sim_track.append(new_cone)

        parsed_orange_cones = self.parse_orange_cones(oranges)
        if len(parsed_orange_cones) == 0:
            return
        blues.insert(0, parsed_orange_cones[0])
        blues.append(parsed_orange_cones[1])
        yellows.insert(0, parsed_orange_cones[2])
        yellows.append(parsed_orange_cones[3])

        # Sort the blue and yellow cones starting from the far orange cone, 
        # and ending at the close orange cone.
        blue_route = two_opt(np.asarray(blues), 0.001)
        yellow_route = two_opt(np.asarray(yellows), 0.001)

        blue_x = [blues[blue_route[i]][0] for i in range(len(blue_route))]
        blue_y = [blues[blue_route[i]][1] for i in range(len(blue_route))]
        yellow_x = [yellows[yellow_route[i]][0] for i in range(len(yellow_route))]
        yellow_y = [yellows[yellow_route[i]][1] for i in range(len(yellow_route))]

        # Check that the cones are ordered correctly, that they move, at least partially, 
        # positively along the x axis, if they don't, reverse the order to correct it.
        if blue_x[1] < blue_x[0]: 
            blue_x = list(reversed(blue_x))
            blue_y = list(reversed(blue_y))

        if yellow_x[1] < yellow_x[0]: 
            yellow_x = list(reversed(yellow_x))
            yellow_y = list(reversed(yellow_y))

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

    def parse_orange_cones(self, orange_cones: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Breaks the big orange starting cones into their position relative to the other blue/yellow cones. 
        Returns format close_blue, far_blue, close_yellow, far_yellow. 
        """
        if len(orange_cones) != 4:
            self.get_logger().fatal('parse_orange_cones called with less than 4 visible cones. Requires 4 cones.')
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
