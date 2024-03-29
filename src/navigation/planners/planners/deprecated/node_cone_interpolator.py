from math import atan2, cos, dist, pi, sin

import numpy as np
import scipy.spatial
from transforms3d.euler import quat2euler

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from driverless_msgs.msg import Cone, ConeDetectionStamped, PathPoint

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

        # List of original cones
        ordered_cones: List[Cone] = orderedCone_msg.cones

        # increment numPoints_interpolated for divisions (n points = n + 1 divisions)
        numPoints_divisions = self.numPoints_interpolated + 1

        # Get length of ordered_cones list
        ordered_cones_length = len(ordered_cones)

        # Get highest 'order' value for blue and yellow cones
        largestBlue = 0
        largestYellow = 0
        for cone in range(ordered_cones_length):
            if (ordered_cones[cone].color == Cone.BLUE) & (ordered_cones[cone].order > largestBlue):
                largestBlue = ordered_cones[cone].order
            elif (ordered_cones[cone].color == Cone.YELLOW) & (ordered_cones[cone].order > largestYellow):
                largestYellow = ordered_cones[cone].order

        # Interpolate cones between pairs of cones along a straight line
        interpolated_cones = []
        for cone in range(ordered_cones_length):
            nextCone = cone + 1
            # Interpolate between last and first cones when at end of cones on one side of the track
            if (ordered_cones[cone].color == Cone.BLUE) & (cone == largestBlue):
                nextCone = 0
            elif (ordered_cones[cone].color == Cone.YELLOW) & (cone == ordered_cones_length - 1):
                nextCone = largestBlue + 1

            # get distance between two ordered cones
            firstCone_XY = ordered_cones[cone].location.x, ordered_cones[cone].location.y
            secondCone_XY = ordered_cones[nextCone].location.x, ordered_cones[nextCone].location.y
            conePair_dist = dist(firstCone_XY, secondCone_XY)

            # get sub-distance between cones for distance between interpolated cones and real cones
            interpolation_subDist = conePair_dist / numPoints_divisions

            # get angle between original cone pair so interpolated cones follow same angle
            conePair_radians = angle(firstCone_XY, secondCone_XY)

            # generate interpolated cone locations
            iCone_locations = []
            cone_of_origin = firstCone_XY
            for iCone in range(self.numPoints_interpolated):
                iCone_coords = new_coordinates(
                    cone_of_origin[0], cone_of_origin[1], conePair_radians, interpolation_subDist
                )
                iCone_locations.append(iCone_coords)

                cone_of_origin = iCone_locations[iCone]

            # create list of interpolated cones with interpolated locations
            for iCone in range(len(iCone_locations)):
                # create new cone
                interpolatedCone = Cone()
                interpolatedCone.location.x = iCone_locations[iCone][0]
                interpolatedCone.location.y = iCone_locations[iCone][1]
                interpolatedCone.color = ordered_cones[cone].color
                interpolatedCone.track_side = ordered_cones[cone].track_side
                interpolatedCone.order = 0  # A dummy value for now - edited later

                # add cone to list of interpolated cones
                interpolated_cones.append(interpolatedCone)

            # Append each interpolated cone to the list of all cones
            # for cone in range(len(interpolated_cones)):
            #     ordered_cones.append(interpolated_cones[cone])

        # Create a new ordered list of all cones, adding blue then yellow cones
        newOrderedCones = []
        newOrder = 0

        for cone in range(len(ordered_cones)):
            if ordered_cones[cone].color == Cone.YELLOW:
                continue

            # add one real cone
            ordered_cones[cone].order = newOrder
            newOrder += 1
            newOrderedCones.append(ordered_cones[cone])

            # add each following interpolated cone
            for iCone in range(self.numPoints_interpolated):
                iConeIndex = cone * self.numPoints_interpolated + iCone
                interpolated_cones[iConeIndex].order = newOrder
                newOrder += 1
                newOrderedCones.append(interpolated_cones[iConeIndex])

            if ordered_cones[cone].order == largestBlue:
                continue

        newOrder = 0
        for cone in range(len(ordered_cones)):
            if ordered_cones[cone].color == Cone.BLUE:
                continue

            # add one real cone
            ordered_cones[cone].order = newOrder
            newOrder += 1
            newOrderedCones.append(ordered_cones[cone])

            # add each following interpolated cone
            for iCone in range(self.numPoints_interpolated):
                iConeIndex = cone * self.numPoints_interpolated + iCone
                interpolated_cones[iConeIndex].order = newOrder
                newOrder += 1
                newOrderedCones.append(interpolated_cones[iConeIndex])

            if ordered_cones[cone].order == largestYellow:
                continue

        # Publish list of ordered and interpolated cones
        interpolatedCones_msg = ConeDetectionStamped(cones=newOrderedCones)
        self.interpolatedCones_publisher.publish(interpolatedCones_msg)

        # ======================================================
        #                   Debugging Code
        # ======================================================

        # Graphing:
        # import matplotlib.pyplot as plt

        # # Separate the x and y coordinates
        # x = [cone.location.x for cone in newOrderedCones]
        # y = [cone.location.y for cone in newOrderedCones]

        # # Plot the points
        # plt.scatter(x, y)

        # # Add labels and a title
        # plt.xlabel("x - axis")
        # plt.ylabel("y - axis")
        # plt.title("2D Plane Plot of Ordered Cones")

        # # Display the plot
        # plt.show()


def main(args=None):
    # begin ros node
    rclpy.init(args=args)
    node = ConeInterpolator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
