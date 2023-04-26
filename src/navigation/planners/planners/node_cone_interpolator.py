from math import atan2, cos, dist, pi, sin

import numpy as np
import scipy.spatial
from transforms3d.euler import quat2euler

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from driverless_msgs.msg import Cone, ConeDetectionStamped

from typing import List


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


class ConeInterpolator(Node):
    """
    This node takes an ordered list of cones and interpolates imaginary cones between real cones to provide track
    boundaries with higher definition.
    """

    # --------------------------------
    # Constants:
    numPoints_interpolated = 3  # number of points interpolated between each pair of cones

    def __init__(self):
        super().__init__("cone_interpolator")

        # Subscribers:
        # sub to list of ordered cones
        self.create_subscription(ConeDetectionStamped, "/planner/ordered_map", self.map_callback, 10)

        # Publishers:
        # publish list of ordered original and interpolated cones
        self.interpolatedCones_publisher: Publisher = self.create_publisher(
            ConeDetectionStamped, "/planner/interpolated_map", 10
        )

        self.get_logger().info("---Cone Interpolator Node Initalised---")
        print("The node is initialised!")

    def map_callback(self, orderedCone_msg: ConeDetectionStamped):
        self.get_logger().debug("Received map")

        ordered_cones: List[Cone] = orderedCone_msg.cones

        # increment numPoints_interpolated for divisions (n points = n + 1 divisions)
        numPoints_divisions = self.numPoints_interpolated + 1

        # Get length of ordered_cones list for pair wise traversal (-1)
        ordered_cones_length = range(ordered_cones) - 1

        # Interpolate cones between pairs of cones along a straight line
        for cone in ordered_cones_length:
            # print(ordered_cones[cone].location.x)
            # print(ordered_cones[cone + 1].location.x)

            # make sure cones on same side of track, skip otherwise
            if ordered_cones[cone].track_side != ordered_cones[cone + 1].track_side:
                continue

            # get distance between two ordered cones
            firstCone_XY = ordered_cones[cone][0][0:1]
            secondCone_XY = ordered_cones[cone + 1][0][0:1]
            # print(str(firstCone_XY))
            # print(str(secondCone_XY))
            conePair_dist = dist(firstCone_XY, secondCone_XY)

            # get sub-distance between cones for distance between interpolated cones and real cones
            interpolation_subDist = conePair_dist / numPoints_divisions

            # get angle between original cone pair so interpolated cones follow same angle
            conePair_radians = angle(firstCone_XY, secondCone_XY)

            # generate interpolated cone locations
            iCone_locations = []
            cone_of_origin = firstCone_XY
            for iCone in range(self.numPoints_interpolated):
                iCone_locations[iCone] = new_coordinates(
                    cone_of_origin[0], cone_of_origin[1], conePair_radians, interpolation_subDist
                )

                cone_of_origin = iCone_locations[iCone]

            # create list of interpolated cones with interpolated locations
            interpolated_cones = []
            for iCone in range(iCone_locations):
                # create new cone
                interpolatedCone = Cone()
                interpolatedCone.location = iCone_locations[iCone]
                interpolatedCone.color = ordered_cones[cone].color
                interpolatedCone.track_side = ordered_cones[cone].track_side

                # add cone to list of interpolated cones
                interpolated_cones[iCone] = interpolatedCone

            # add list of interpolated cones to list of all ordered cones
            insert_index = cone + 1
            ordered_cones[insert_index:insert_index] = interpolated_cones

        # Publish list of ordered and interpolated cones
        interpolatedCones_msg = ConeDetectionStamped(cones=ordered_cones)
        self.interpolatedCones_publisher.publish(interpolatedCones_msg)


def main(args=None):
    # begin ros node
    rclpy.init(args=args)
    node = ConeInterpolator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
