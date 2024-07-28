from math import atan2, sqrt
import time
from geometry_msgs.msg import Quaternion
from transforms3d.euler import quat2euler, euler2quat

import numpy as np

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from typing import List, Tuple

# https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html
# This article explains what QoS (quality of service) is and how to use it properly. Simply put, QoS policies give us
# control over how publishers and subscribers manage messages up until they are processed. Internally, subscribers store
# the messages they receive in a queue (the data structure), when the subscriber finishes processing a message, it
# processes the next message in the queue. As queues are FIFO, the subscriber processes older messages before newer
# messages, this is not always the behaviour we want. For example, when using the car's pose to calculate the next
# steering angle, we only ever want to be using the latest pose. The default behaviour of a subscriber is to store the
# previous 10 messages, in this example, that would mean we calculate the steering angle for where the car was 10
# messages ago, and apply it to where the car currently is, leading to the car following a  delayed version of the
# track.

# This profile will only store one message in its internal queue, meaning it only holds onto the latest message.
# You should only use this when you only want to process the latest message, and don't care about any messages that you
# may miss while processing another message. For example, when using car pose to calculate where to steer in order to
# follow the track, you only want to be using the latest car pose.
QOS_LATEST = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE,
)

# This profile will store up to 100 messages in its internal queue. You should use this profile when you need to process
# all messages sent by the publisher in the order that they are received. For example, /vision/cone_detection
# subscribers should use this profile as they need to process every message in order to see every cone and build a
# correct track layout.
QOS_ALL = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=100,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def dist(p1: List[float], p2: List[float]) -> float:
    """
    Retrieve distance between two points
    * param p1: [x,y] coords of point 1
    * param p2: [x,y] coords of point 2
    * return: distance between points
    """
    return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def fast_dist(p1: List[float], p2: List[float]) -> float:
    """
    Calculates the distance between two 2-dimensional points using the euclidian distance formula, but doesn't sqrt the
    result, therefore the result of this function is the distance between the two points squared. This should only be
    used for operations that compare distances created by this function, for example, if you  have a point, and need to
    find the closest point to it from a list, you should use this function to calculate the distances for each point in
    the list, they can be compared to each other as they are all squared. Sqrts are very expensive calculations, and
    this function allows us to avoid performing them n times for an n sized list.

    Be careful what you compare the reuslt of this function against, if you are checking that for points in a list that
    are over or under a certain distance, you must square the distance before the comparison.
    """
    diff_x = p1[0] - p2[0]
    diff_y = p1[1] - p2[1]
    return diff_x * diff_x + diff_y * diff_y


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


def wrap_to_pi(angle: float) -> float:
    """
    Wrap an angle between -pi and pi
    * param angle: angle in rads
    * return: angle in rads wrapped to -pi and pi
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi


def midpoint(p1: List[float], p2: List[float]) -> Tuple[float]:
    """
    Retrieve midpoint between two points
    * param p1: [x,y] coords of point 1
    * param p2: [x,y] coords of point 2
    * return: x,y tuple of midpoint coord
    """
    return (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2


def yaw(quat: Quaternion):
    """
    Retrieve yaw from quaternion
    * param quat: quaternion
    * return: yaw in rads    
    """

    return quat2euler([quat.w, quat.x, quat.y, quat.z])[2]  # yaw

def orientation(x, y, z):
    """
    Create a quaternion from euler angles
    * param x: roll
    * param y: pitch
    * param z: yaw
    * return: quaternion
    """

    new_orientation = euler2quat(x, y, z)
    return Quaternion(w=new_orientation[0], x=new_orientation[1], y=new_orientation[2], z=new_orientation[3])

class FPSHandler:
    def __init__(self):
        self.timestamp = time.time() + 1
        self.start = time.time()
        self.frame_cnt = 0

    def next_iter(self):
        self.timestamp = time.time()
        self.frame_cnt += 1

    def fps(self):
        return self.frame_cnt / (self.timestamp - self.start)
