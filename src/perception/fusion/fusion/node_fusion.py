import time

import numpy as np
from sklearn.neighbors import KDTree

import message_filters
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from driverless_msgs.msg import Cone, ConeDetectionStamped, ConeWithCovariance
from geometry_msgs.msg import Point
from std_msgs.msg import Header

from driverless_common.cone_props import ConeProps

from typing import List, Tuple


# node class object that gets created
class PerceptionFusion(Node):
    leaf = 10  # nodes per tree before it starts brute forcing?
    radius = 1.3  # nn kdtree nearch

    def __init__(self):
        super().__init__("perception_fusion")
        # cone detection subscribers
        vision_sub = message_filters.Subscriber(self, ConeDetectionStamped, "/vision/cone_detection")
        lidar_sub = message_filters.Subscriber(self, ConeDetectionStamped, "/lidar/cone_detection")
        synchronizer = message_filters.ApproximateTimeSynchronizer(fs=[lidar_sub, vision_sub], queue_size=3, slop=0.2)
        synchronizer.registerCallback(self.callback)

        # potentially sub to odom to correct for motion between vision and lidar
        # may use a msg cache for storing a number of odom msg then match one to vision stamp and one to lidar stamp

        self.fusion_publisher: Publisher = self.create_publisher(ConeDetectionStamped, "/fusion/cone_detection", 1)

        self.get_logger().info("---Perception Fusion Node Initialised---")

    # TODO: implement redundancy to run on just vision if required (cant do just lidar)
    def callback(self, lidar_msg: ConeDetectionStamped, vision_msg: ConeDetectionStamped):
        self.get_logger().debug("Received detection")
        start: float = time.time()

        # process detected cones
        vision_cones: List[Cone] = vision_msg.cones
        lidar_cones: List[Cone] = lidar_msg.cones

        # get list of lidar cone locations
        # make each cone with covariance (lidar is pretty accurate) with UNKNOWN colour
        # make a neighbourhood?? need to work out better way to query distance
        # transform vision detection to lidar location
        # query for closest lidar cone
        # found closest lidar cone
        #   update assign colour from vision
        #   update to some combined position between vision and lidar, with lidar position weighted higher
        # not found
        #   make a cone with covariance (less accurate from vision) with colour

        cones_with_cov: np.ndarray = []
        for lidar_cone in lidar_cones:
            lidar_cone.location.x = lidar_cone.location.x + 1.65
            # this is the distance between lidar and baselink hard-coded
            # TODO: better way of transforming?

            # arbitrary covariance TODO: better covariance so SLAM can actually use this
            cone = ConeProps(lidar_cone)
            cov = [cone.range * 0.01, cone.range * 0.02]  # xx,yy covariances
            cone_with_cov = [lidar_cone.location.x, lidar_cone.location.y, lidar_cone.color, cov[0], cov[1]]

            if cones_with_cov == []:  # first in this list
                cones_with_cov = np.array(cone_with_cov)
                cones_with_cov = np.reshape(cones_with_cov, (1, -1))  # turn 2D
            else:
                cones_with_cov = np.vstack([cones_with_cov, cone_with_cov])

        if len(cones_with_cov) != 0:
            neighbourhood = KDTree(cones_with_cov[:, :2], leaf_size=self.leaf)

        for vision_cone in vision_cones:
            vision_cone.location.x = vision_cone.location.x - 0.1  #  distance hard-coded

            check = np.reshape([vision_cone.location.x, vision_cone.location.y], (1, -1))  # turn into a 2D row array
            data = neighbourhood.query_radius(check, r=self.radius)  # check neighbours in radius
            index = data[0]  # index from query data (this was a pain to find what the data outputs)

            if index.size != 0:  # if there is an index for closest cone in the lidar list
                cones_with_cov[index, 2] = vision_cone.color  # overwrite that cone's colour
                # make a more accurate covariance based on lidar-vision distance
                dist_x = cones_with_cov[index, 0] - vision_cone.location.x
                dist_y = cones_with_cov[index, 1] - vision_cone.location.y
                # xx,yy covariances
                cones_with_cov[index, 3] = dist_x * 1  # proportional constant
                cones_with_cov[index, 4] = dist_y * 1

            else:
                # otherwise make a new cone with covariance based on known inaccuracies in vision
                cone = ConeProps(vision_cone)
                cov = [cone.range * 0.3**1.5, cone.bearing * 0.4**1.5]  # xx,yy covariances
                cone_with_cov = [vision_cone.location.x, vision_cone.location.y, vision_cone.color, cov[0], cov[1]]
                cones_with_cov = np.vstack([cones_with_cov, cone_with_cov])

        # new detection with respect to car base_link (so matches odom location)
        cone_detection_msg = ConeDetectionStamped(header=Header(frame_id="base_link", stamp=vision_msg.header.stamp))
        for cone_with_cov in cones_with_cov:
            cone_with_cov_msg = ConeWithCovariance()
            cone_with_cov_msg.cone = Cone(
                location=Point(x=cone_with_cov[0], y=cone_with_cov[1], z=0.0),
                color=int(cone_with_cov[2]),
            )
            cone_with_cov_msg.covariance = [cone_with_cov[3], 0.0, 0.0, cone_with_cov[4]]  # diagonal square matrix
            cone_detection_msg.cones_with_cov.append(cone_with_cov_msg)

        self.fusion_publisher.publish(cone_detection_msg)

        self.get_logger().debug(f"Total time: {str(time.time() - start)}")


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
