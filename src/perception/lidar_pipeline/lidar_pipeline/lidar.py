# import ROS2 libraries
from math import fabs
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
# import ROS2 message libraries
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
# import custom message libraries
from driverless_msgs.msg import Cone, ConeDetectionStamped

# other python libraries
import time
import numpy as np
from typing import List

# import ROS function that has been ported to ROS2 by
# SebastianGrans https://github.com/SebastianGrans/ROS2-Point-Cloud-Demo
from .sub_module.read_pcl import *
# lidar cone detection algorithm
from .sub_module.ground_plane_estimation import lidar_main

# LIDAR_NODE = '/fsds/lidar/Lidar1'
LIDAR_NODE = '/velodyne_points'

DISPLAY = True
VISUALISE = True
MAX_RANGE = 13 #m

def cone_msg(x_coord: float, y_coord: float) -> Cone: 
    # {Cone.YELLOW, Cone.BLUE, Cone.ORANGE_SMALL}
    location = Point(
        x=x_coord,
        y=y_coord,
        z=0.2,
    )

    return Cone(
        location=location,
        color=4,
    )


class LidarDetection(Node):
    def __init__(self):
        super().__init__('lidar_detector')
        logger = self.get_logger()

        if LIDAR_NODE == '/fsds/lidar/Lidar1':
            logger.info("LiDAR Processing for FSDS")
        elif LIDAR_NODE == '/velodyne_points':
            logger.info("LiDAR Processing for Velodyne")

        self.pcl_subscription = self.create_subscription(
            PointCloud2,
            LIDAR_NODE,
            self.pcl_callback,
            10)
        self.pcl_subscription  # prevent unused variable warning

        self.detection_publisher: Publisher = self.create_publisher(
            ConeDetectionStamped, 
            "lidar_detector/cone_detection", 
            1)


    ## callback for lidar data to be sent to. used to call funtion to find cone coords
    def pcl_callback(self, pcl_msg: PointCloud2):
        """ In here, we will call calculations to get the xyz location 
        and reflectivity of the cones"""

        logger = self.get_logger()
        
        start: float = time.time()
        # Convert the list of floats into a list of xyz coordinates

        point_tuples: list = read_points_list(pcl_msg, skip_nans=True)
        point_list: list = []
        for point in point_tuples:
            if point[0] > 0:
                if LIDAR_NODE == '/fsds/lidar/Lidar1':
                    point_list.append([point[0], point[1], point[2]])
                elif LIDAR_NODE == '/velodyne_points':
                    point_list.append([point[0], point[1], point[2]])
        # logger.info("convert time: " + str(time.time()-start))

        # with open('/home/developer/datasets/points1.txt', 'w') as f:
        #     f.write(str(point_list))
        # logger.info("wrote points")

        # calls main module from ground estimation algorithm
        cones: list = lidar_main(point_list, DISPLAY, VISUALISE, "/home/developer/datasets/figures", MAX_RANGE) 

        logger.info("Algorithm Time:" + str(time.time()-start))

        # define message component - list of Cone type messages
        detected_cones: List[Cone] = [] 
        for i in range(len(cones)):
            # add cone to msg list
            detected_cones.append(cone_msg(cones[i][0], cones[i][1]))
            print(cones[i][0], cones[i][1])
    
        detection_msg = ConeDetectionStamped(
            header=pcl_msg.header,
            cones=detected_cones
        )

        self.detection_publisher.publish(detection_msg) # publish cone data
        

## main call
def main(args=None):
    rclpy.init(args=args)

    detection_node = LidarDetection()
    rclpy.spin(detection_node)
    
    detection_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()