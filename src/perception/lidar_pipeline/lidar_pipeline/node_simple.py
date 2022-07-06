# import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
# import ROS2 message libraries
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
# import custom message libraries
from driverless_msgs.msg import Cone, ConeDetectionStamped

# other python modules
import time
import numpy as np
from typing import List
from math import sin, cos, sqrt, atan2

# import ROS function that has been ported to ROS2 by
# SebastianGrans https://github.com/SebastianGrans/ROS2-Point-Cloud-Demo
from .scripts.read_pcl import read_points_list
# lidar cone detection algorithm
from .scripts.sim_simple import find_cones


"""
covariance matrix determined by comparing
ground-truth sim cone locations with lidar cone locations
- performed in a debug trial+error node
- probably wont work for real life but thats ok,
  there are better options with SLAM
"""
lidar_cov = np.array([[0.04, 0, 0], [ 0, 0.06, 0], [0, 0, 0.02]])
def cone_cov(x: float, y: float, z: float):
    bearing = atan2(y, x)
    distance = sqrt(x**2 +y**2 + z**2)
    s, c = sin(bearing), cos(bearing)
    rotation_matrix = np.array([[c, -1*s, 0],[s, c, 0], [0, 0, 1]])
    new_cov = rotation_matrix @ lidar_cov @ rotation_matrix.T
    retcov = new_cov * distance/20
    return retcov


def cone_msg(x_coord: float, y_coord: float, z_coord: float) -> Cone: 
    location: Point = Point(
        x=x_coord+1.2, # distance from lidar to CoG
        y=y_coord,
        z=z_coord+0.2, # scanned height + height of lidar from ground
    )
    cov = cone_cov(x_coord, y_coord, z_coord)

    return Cone(
        location=location,
        covariance=cov.flatten(),
        color=4,
    )


class LidarProcessing(Node):
    def __init__(self):
        super().__init__('lidar_processor')

        self.create_subscription(PointCloud2, "/lidar/Lidar1", self.callback, 10)

        self.detection_publisher: Publisher = self.create_publisher(ConeDetectionStamped, "lidar/cone_detection", 1)

        self.get_logger().info('---LiDAR sim processing node initialised---')


    def callback(self, pc2_msg: PointCloud2):
        """ 
        lidar point cloud message sent here. 
        used to call funtions to find cone coords.
        to get the xyz location and colour of cones.
        """
        logger = self.get_logger()
        
        start: float = time.time()
        
        # Convert the list of floats into a list of xyz coordinates
        point_array: List[List] = read_points_list(pc2_msg)

        logger.debug("Read Time:" + str(time.time()-start))

        # calls main module from ground estimation algorithm
        cones: List[List] = find_cones(point_array) 

        logger.debug("Detected cones:" + str(len(cones)))
        
        # define message component - list of Cone type messages
        detected_cones: List[Cone] = []

        for cone in cones:
            detected_cones.append(cone_msg(cone[0], cone[1], cone[2]))

        stamped_header = Header()
        stamped_header.stamp = pc2_msg.header.stamp
        stamped_header.frame_id = "detection"
        detection_msg = ConeDetectionStamped(
            header=stamped_header,
            cones=detected_cones
        )

        self.detection_publisher.publish(detection_msg) # publish cone data

        logger.info("Total Time:" + str(time.time()-start) + "\n")


def main():
    # begin ros node
    rclpy.init()

    node = LidarProcessing()
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()

