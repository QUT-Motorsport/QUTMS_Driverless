from math import atan2, cos, dist, pi, sin, sqrt

import numpy as np
import scipy.interpolate as scipy_interpolate

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from driverless_msgs.msg import (
    Cone,
    ConeDetectionStamped,
    ConeWithCovariance,
    PathPoint,
    PathStamped,
    TrackDetectionStamped,
)
from geometry_msgs.msg import Point

from typing import List, Tuple

# from fs_msgs.msg import Track


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


def new_coordinates(x, y, angle_rads, distance):
    """
    Calculate new coordinates after moving from an initial point in a specified direction and distance.

    * param x: float, x-coordinate of the initial point
    * param y: float, y-coordinate of the initial point
    * param angle_rads: float, angle in radians to move from initial point
    * param distance: float, distance to move from the initial point
    * return: tuple (x_new, y_new, 0), new coordinates after moving
    """
    x_new = x + distance * cos(angle_rads)
    y_new = y + distance * sin(angle_rads)
    return (x_new, y_new, 0)


def interpolate(subDistance, pointAngle, firstPoint_pos, numPoints):
    """
    Calculate the interpolated positions between a series of points on a curve defined by
    an angle (pointAngle) and positions (firstPoint_pos).

    * param subDistance: float, the distance between consecutive interpolated points
    * param pointAngle: the angles between the points in radians
    * param firstPoint_pos: tuple (x, y), the initial position of the first point
    * param numPoints: int, the number of interpolated points to calculate along the curve

    * return: list of tuples (x, y), the calculated interpolated positions
    """
    iCone_locations = []
    cone_of_origin = firstPoint_pos
    for iCone in range(int(numPoints)):
        iCone_coords = new_coordinates(cone_of_origin[0], cone_of_origin[1], pointAngle, subDistance)
        iCone_locations.append(iCone_coords)

        cone_of_origin = iCone_locations[iCone]

    return iCone_locations


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

    cone_count = len(cones)
    mat = [[0] * cone_count for i in range(cone_count)]
    for i, a in enumerate(cones):
        for j, b in enumerate(cones):
            if i == j:
                continue
            # Prevent unnecessary square root calculations. We don't need to
            # calculate distance if the distance is going to be greater than 5m (sqrt(25)).
            # So don't square root, or store the distance in the mat.
            # Note: Doubled the accepted distance to account for human error and camera noise.
            diff = [a[0] - b[0], a[1] - b[1]]
            dist = diff[0] * diff[0] + diff[1] * diff[1]
            if dist <= 50:
                mat[i][j] = sqrt(dist)

    order = [start_index]
    # Loop for each cone that needs to be ordered.
    # -2 accounts for the start and end indices that are manually handled.
    for unused in range(len(mat) - 2):
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


class EquidistantMidline(Node):
    DISTANCE_BETWEEN_POINTS = 0.01  # Distance between midpoints in meters

    def __init__(self):
        super().__init__("Equidistant_Midline_Node")

        # Subscribers:
        # sub to list of ordered cones
        self.create_subscription(ConeDetectionStamped, "/planner/ordered_map", self.map_callback, 10)

        # Publishers:
        # publish list of 1cm equidistant midline points
        self.path_publisher: Publisher = self.create_publisher(PathStamped, "/planner/path", 1)

        self.get_logger().info("---Sim Path Planner Node Initalised---")

    def map_callback(self, orderedCone_msg: ConeDetectionStamped):
        self.get_logger().debug("Received map")

        # List of original cones
        ordered_cones: List[Cone] = orderedCone_msg.cones

        # Create list for blue and yellow cones
        blues = []
        yellows = []

        # Iterate through all cones and add them to lists by colour
        for cone in ordered_cones:
            if cone.color == Cone.BLUE:
                blues.append([cone.location.x, cone.location.y, 0.0])
            elif cone.color == Cone.YELLOW:
                yellows.append([cone.location.x, cone.location.y, 0.0])

        # Create a list to store path midpoint coordinates
        pathMidPoint_coords = []

        # Iterate through each blue cone and find their nearest yellow cone
        # (can be optimised by restrciting search by index +/- an arbitrary value on yellows)
        for blue_cone in range(len(blues)):
            minimum_dist = 1000
            closestYellow_index = 1000
            for yellow_cone in range(len(yellows)):
                if dist(blues[blue_cone], yellows[yellow_cone]) < minimum_dist:
                    minimum_dist = dist(blues[blue_cone], yellows[yellow_cone])
                    closestYellow_index = yellow_cone

            # For each pair identified by minimum distance to each other, create a midpoint
            pathMidPoint = midpoint(blues[blue_cone], yellows[closestYellow_index])
            pathMidPoint_coords.append(pathMidPoint)

        # Iterate through each pathMidPoint and interpolate equidistant midpoints
        num_pathMidPoints = len(pathMidPoint_coords)
        for pathMidPoint in range(num_pathMidPoints):
            next_pathMidPoint = pathMidPoint + 1

            # Interpolate between last and first pathMidPoint when at end of list
            if pathMidPoint == num_pathMidPoints:
                next_pathMidPoint = 0

            # Get subdistance between interpolated midPoints
            subDistance = self.DISTANCE_BETWEEN_POINTS

            # Get angle between original midPoints
            first_midPoint = pathMidPoint_coords[pathMidPoint]
            second_midPoint = pathMidPoint_coords[next_pathMidPoint]
            midPoint_radians = angle(first_midPoint, second_midPoint)

            # Get number of points to be interpolated between the two midPoints
            first_midPoint_XY = (first_midPoint[0], first_midPoint[1])
            second_midPoint_XY = (second_midPoint[0], second_midPoint[1])
            numPoints = dist(first_midPoint_XY, second_midPoint_XY) / subDistance

            # Get interpolated midPoint coordinates
            interpolated_midPoints = interpolate(subDistance, midPoint_radians, first_midPoint, numPoints)

            # Add each interpolated midPoint to list of all midPoints
            for iMidPoint in interpolated_midPoints:
                pathMidPoint_coords.append(iMidPoint)

        # Create a list of PathPoints to represent all midPoints
        path: list[PathPoint] = []
        for point in range(len(pathMidPoint_coords)):
            path_point = PathPoint()
            path_point.location.x = pathMidPoint_coords[point][0]
            path_point.location.y = pathMidPoint_coords[point][1]
            path_point.location.z = 0.0
            path.append(path_point)

        # Add the first path point to the end of the list to complete the loop
        path.append(path[0])

        # Publish midPoint path
        path_msg = PathStamped(path=path)
        path_msg.header.frame_id = "track"
        self.path_publisher.publish(path_msg)


def main(args=None):
    # begin ros node
    rclpy.init(args=args)
    node = EquidistantMidline()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
