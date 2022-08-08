# import ros2 libraries
import numpy as np
from sklearn.neighbors import KDTree

import message_filters
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

# import custom message libraries
from driverless_msgs.msg import Cone, ConeDetectionStamped, ConeWithCovariance

from typing import List, Tuple


# node class object that gets created
class PerceptionFusion(Node):
    leaf = 30  # nodes per tree before it starts brute forcing?
    radius = 3  # nn kdtree nearch

    def __init__(self):
        super().__init__("perception_fusion")
        # cone detection subscribers
        vision_sub = message_filters.Subscriber(self, ConeDetectionStamped, "/vision/cone_detection")
        lidar_sub = message_filters.Subscriber(self, ConeDetectionStamped, "/lidar/cone_detection")
        synchronizer = message_filters.ApproximateTimeSynchronizer(fs=[lidar_sub, vision_sub], queue_size=10, slop=0.2)
        synchronizer.registerCallback(self.callback)

        # potentially sub to odom to correct for motion between vision and lidar
        # may use a msg cache for storing a number of odom msg then match one to vision stamp and one to lidar stamp

    # function that is called each time the subscriber reads a new message on the topic
    def callback_function_name(self, vision_msg: ConeDetectionStamped, lidar_msg: ConeDetectionStamped):
        self.get_logger().debug("Received detection")

        # process detected cones
        vision_cones: List[Cone] = vision_msg.cones
        lidar_cones: List[Cone] = lidar_msg.cones

        # get list of lidar cone locations
        # make a neighbourhood
        # get list of vision cone locations + colours
        # transform to lidar location
        # query for closest lidar point
        # if found, make a new point with covariance, with lidar position weighted higher
        #   assign colour from vision
        # not found, make

        # get lidar cones into a KDtree neighbourhood
        # lidar_nn: np.ndarray = []
        # for cone in lidar_cones:

        # for cone in vision_cones:
        #     neighbourhood = KDTree(, leaf_size=self.leaf)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
