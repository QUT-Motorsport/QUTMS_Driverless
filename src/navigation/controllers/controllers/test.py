# to run file only:
# python -u /home/daming/QUTMS_Driverless/src/navigation/controllers/controllers/test.py

from math import cos, sin, dist, atan2, pi, sqrt

import numpy as np
import scipy.spatial
from transforms3d.euler import quat2euler

import message_filters
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDrive
from driverless_msgs.msg import PathStamped, Reset, TrackDetectionStamped, Cone, ConeWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped

from typing import List


cone_list = [(1,1), (3,3), (1.5, 1.5), (2,2)]

def interpolate_cones(cone_list: List[float]) -> List[float]:
    """
    Interpolates imaginary cones between consecutive cones
    * param cone_list: List[x,y] coords of all cones on only one side of the track
    * return: List[x,y] coords of all cones on ones side of the track, including interpolated cones
    """

    imaginaryCone_list = []         # ((x,y),index)

    for index, cone in enumerate(cone_list):
        # ensure indexing is within range of list
        if index == len(cone_list) - 1:
            break
        
        # get x,y coords of two consecutive cones
        cone_x = cone[0]
        cone_y = cone[1]
        nextCone_x = cone_list[index+1][0]
        nextCone_y = cone_list[index+1][1]

        # interpolate imaginary cones coords
        imaginaryCone_x = (cone_x + nextCone_x) / 2
        imaginaryCone_y = (cone_y + nextCone_y) / 2

        # add imaginary cones coords to imaginaryCone_list with relevant indexing
        imaginaryCone_tuple = ((imaginaryCone_x, imaginaryCone_y), index+1)
        imaginaryCone_list.append(imaginaryCone_tuple)

    # add all interpolated cones to full list of all cones
    for cone in reversed(imaginaryCone_list):
        cone_list.insert(cone[1], cone[0])

    return cone_list


interpolated_cone_track = interpolate_cones(cone_list)
print(cone_list)
print(interpolated_cone_track)

