from math import sqrt

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from driverless_msgs.msg import Cone, ConeDetectionStamped, ConeWithCovariance
from geometry_msgs.msg import Point

from typing import List


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


def parse_orange_cones(self, orange_cones: List[List[float]]) -> List[List[float]]:
    """
    Breaks the big orange starting cones into their position relative to the other blue/yellow cones.
    Returns format close_blue, far_blue, close_yellow, far_yellow.
    """
    if len(orange_cones) != 4:
        self.get_logger().fatal("parse_orange_cones called with less than 4 visible cones. Requires 4 cones.")
        return []

    # seperate orange cones into track sides by position relative to origin
    # blue_cones & yellow_cones must have two cones each
    #   left    (blue) = +ve y
    #   right (yellow) = -ve y
    blue_cones = [cone for cone in orange_cones if cone[1] > 0]
    yellow_cones = [cone for cone in orange_cones if cone[1] < 0]

    # further seperate left and right cones into near and far
    #   near = smaller x
    #   far  = greater x
    if blue_cones[0][0] > blue_cones[1][0]:
        close_blue, far_blue = blue_cones[1], blue_cones[0]
    else:
        close_blue, far_blue = blue_cones[0], blue_cones[1]

    if yellow_cones[0][0] > yellow_cones[1][0]:
        close_yellow, far_yellow = yellow_cones[1], yellow_cones[0]
    else:
        close_yellow, far_yellow = yellow_cones[0], yellow_cones[1]

    return [close_blue, far_blue, close_yellow, far_yellow]


class OrderedCones(Node):
    def __init__(self):
        super().__init__("Orderd_Cones_Node")

        # sub to track for all cone locations relative to car start point
        self.create_subscription(ConeDetectionStamped, "/ground_truth/global_map", self.map_callback, 10)

        # publishers for ordered cones
        self.orderedCone_publisher: Publisher = self.create_publisher(ConeDetectionStamped, "/planner/ordered_map", 1)

        self.get_logger().info("---Sim Ordered Cones Node Initalised---")

    def map_callback(self, track_msg: ConeDetectionStamped):
        self.get_logger().debug("Received map")

        # Create a list of cones with covariance
        cones_w_cov: List[ConeWithCovariance] = track_msg.cones_with_cov

        # Create lists for each type of cone by colour
        yellows: List[List[float]] = []
        blues: List[List[float]] = []
        oranges: List[List[float]] = []

        # Iterate through each cone, adding them to lists by colour
        for cone_w_cov in cones_w_cov:
            if cone_w_cov.cone.color == Cone.YELLOW:
                yellows.append([cone_w_cov.cone.location.x, cone_w_cov.cone.location.y, 0.0])
            elif cone_w_cov.cone.color == Cone.BLUE:
                blues.append([cone_w_cov.cone.location.x, cone_w_cov.cone.location.y, 0.0])
            elif cone_w_cov.cone.color == Cone.ORANGE_BIG:
                oranges.append([cone_w_cov.cone.location.x, cone_w_cov.cone.location.y])

        # Use the orange cones as start and end of each track side
        parsed_orange_cones = parse_orange_cones(oranges)
        if len(parsed_orange_cones) == 0:
            return

        # Treat orange cones as blue/yellow cones
        blues_first = parsed_orange_cones[1]
        blues_last = parsed_orange_cones[0]
        yellows_first = parsed_orange_cones[3]
        yellows_last = parsed_orange_cones[2]

        # Add 'z' coordinate of 0.0 to each orange cone
        blues_first.append(0.0)
        blues_last.append(0.0)
        yellows_first.append(0.0)
        yellows_last.append(0.0)

        # Add orange cones to corresponding locations in blue/yellow cone lists
        blues.insert(0, blues_first)
        blues.append(blues_last)
        yellows.insert(0, yellows_first)
        yellows.append(yellows_last)

        # Sort the blue and yellow cones starting from the far orange cone, and ending at the close orange cone.
        ordered_blues = sort_cones(blues)
        ordered_yellows = sort_cones(yellows)

        # Add each cone from ordered_blues & ordered_yellows to ordered_cones
        # ordered_cones: [location (x,y,z), color (b=0,y=4), order, track_side (b=0,y=1)]
        ordered_cones: List[Cone] = []
        for cone in range(len(ordered_blues) - 1):
            ordered_cones.append(
                Cone(
                    location=Point(  # point (x,y,z)
                        x=ordered_blues[cone][0], y=ordered_blues[cone][1], z=ordered_blues[cone][2]
                    ),
                    color=0,  # color (b=0,y=4)
                    order=cone,  # order
                    track_side=0,  # track_side (b=0,y=1)
                )
            )
        for cone in range(len(ordered_yellows) - 1):
            ordered_cones.append(
                Cone(
                    location=Point(  # point (x,y,z)
                        x=ordered_yellows[cone][0], y=ordered_yellows[cone][1], z=ordered_yellows[cone][2]
                    ),
                    color=4,  # color (b=0,y=4)
                    order=cone,  # order
                    track_side=1,  # track_side (b=0,y=1)
                )
            )

        orderedCone_msg = ConeDetectionStamped(cones=ordered_cones)
        self.orderedCone_publisher.publish(orderedCone_msg)


def main(args=None):
    # begin ros node
    rclpy.init(args=args)
    node = OrderedCones()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
