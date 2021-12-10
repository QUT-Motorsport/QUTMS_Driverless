import pickle
import datetime as dt

# import ROS2 libraries
from math import fabs
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
# import ROS2 message libraries
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration, Time
# import custom message libraries
from driverless_msgs.msg import Cone, ConeDetectionStamped

# other python libraries
import time
from typing import List, NamedTuple, Tuple

# import ROS function that has been ported to ROS2 by
# SebastianGrans https://github.com/SebastianGrans/ROS2-Point-Cloud-Demo
from .sub_module.read_pcl import read_points_list
# lidar cone detection algorithm
from .sub_module.ground_plane_estimation import lidar_main, lidar_init

# LIDAR_NODE = '/fsds/lidar/Lidar1'
LIDAR_NODE = '/velodyne_points'

DISPLAY = False
VISUALISE = True
MAX_RANGE = 7.5 #m

def cone_msg(x_coord: float, y_coord: float) -> Cone: 
    # {Cone.YELLOW, Cone.BLUE, Cone.ORANGE_SMALL}
    location = Point(
        x=x_coord,
        y=y_coord,
        z=0.0,
    )

    return Cone(
        location=location,
        color=4,
    )


def marker_msg(x_coord: float, y_coord: float, ID: int, head: Header) -> Marker: 
    marker = Marker()
    marker.header = head
    marker.ns = "current_scan"
    marker.id = ID
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD

    marker.pose.position.x = x_coord
    marker.pose.position.y = y_coord
    marker.pose.position.z = 0.0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # scale out of 1x1x1m
    marker.scale.x = 0.228
    marker.scale.y = 0.228
    marker.scale.z = 0.325

    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    marker.lifetime = Duration(sec=1, nanosec=0)

    return marker

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
        lidar_init(DISPLAY, VISUALISE, "/home/developer/datasets/figures", MAX_RANGE)

        self.detection_publisher: Publisher = self.create_publisher(
            ConeDetectionStamped, 
            "lidar_detector/cone_detection", 
            1)

        self.marker_publisher: Publisher = self.create_publisher(
            MarkerArray, 
            "lidar_detector/debug_cones_array", 
            1)

        self.count: int = 0


    ## callback for lidar data to be sent to. used to call funtion to find cone coords
    def pcl_callback(self, pcl_msg: PointCloud2):
        """ In here, we will call calculations to get the xyz location 
        and reflectivity of the cones"""

        logger = self.get_logger()
        
        start: float = time.time()
        # Convert the list of floats into a list of xyz coordinates

        point_array: List[NamedTuple] = read_points_list(pcl_msg, skip_nans=True)

        with open(f"/home/developer/datasets/reconstruction/{self.count}_pointcloud.txt", 'x') as f:
            f.write(str(point_array))
        self.count += 1
        # logger.info("wrote points")

        # calls main module from ground estimation algorithm
        cones: List[list] = lidar_main(point_array, self.count) 

        # define message component - list of Cone type messages
        detected_cones: List[Cone] = []
        markers_list: List[Marker] = []
        for i in range(len(cones)):
            # add cone to msg list
            detected_cones.append(cone_msg(
                cones[i][0], 
                cones[i][1],
            ))
            markers_list.append(marker_msg(
                cones[i][0], 
                cones[i][1], 
                i, 
                pcl_msg.header,
            ))

        detection_msg = ConeDetectionStamped(
            header=pcl_msg.header,
            cones=detected_cones
        )

        markers_msg = MarkerArray(markers=markers_list)

        self.detection_publisher.publish(detection_msg) # publish cone data
        self.marker_publisher.publish(markers_msg) # publish marker points data

        logger.info("Total Time:" + str(time.time()-start))


## main call
def main(args=None):
    rclpy.init(args=args)

    detection_node = LidarDetection()
    rclpy.spin(detection_node)
    
    detection_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()