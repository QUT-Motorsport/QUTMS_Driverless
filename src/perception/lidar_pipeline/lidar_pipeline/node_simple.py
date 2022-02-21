# import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
# import ROS2 message libraries
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
# import custom message libraries
from driverless_msgs.msg import Cone, ConeDetectionStamped

# other python modules
import numpy
import time
from typing import List
import sys
import os
import getopt
import pathlib

# import ROS function that has been ported to ROS2 by
# SebastianGrans https://github.com/SebastianGrans/ROS2-Point-Cloud-Demo
from .scripts.read_pcl import read_points_list
# lidar cone detection algorithm
from .scripts.sim_simple import find_cones


def cone_msg(x_coord: float, y_coord: float) -> Cone: 
    # {Cone.YELLOW, Cone.BLUE, Cone.ORANGE_SMALL}
    location: Point = Point(
        x=x_coord,
        y=y_coord,
        z=0.0,
    )

    return Cone(
        location=location,
        color=4,
    )


def marker_msg(
    x_coord: float, 
    y_coord: float, 
    ID: int
) -> Marker: 

    marker = Marker()
    marker.header.frame_id = "fsds/FSCar"
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

    marker.lifetime = Duration(sec=0, nanosec=100000000)

    return marker


class LidarProcessing(Node):
    def __init__(self, max_range: int):
        super().__init__('lidar_processor')

        self.create_subscription(PointCloud2, "/lidar/Lidar1", self.callback, 10)

        self.detection_publisher: Publisher = self.create_publisher(ConeDetectionStamped, "lidar/cone_detection", 1)
        self.marker_publisher: Publisher = self.create_publisher(MarkerArray, "lidar/debug_cones_array", 1)

        self.max_range = max_range

        self.get_logger().info('---LiDAR processing node initialised---')


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

        logger.info("Read Time:" + str(time.time()-start))

        # calls main module from ground estimation algorithm
        cones: List[List] = find_cones(point_array) 

        logger.info("Detected cones:" + str(len(cones)))
        
        # define message component - list of Cone type messages
        detected_cones: List[Cone] = []
        markers_list: List[Marker] = []
        for i in range(len(cones)):
            # add cone to msg list
            detected_cones.append(cone_msg(
                cones[i][0], 
                cones[i][1],
            ))
            
            marker = marker_msg(
                cones[i][0], 
                cones[i][1], 
                i, 
            )
            marker.header.stamp = self.get_clock().now().to_msg()
            markers_list.append(marker)

        detection_msg = ConeDetectionStamped(
            header=pc2_msg.header,
            cones=detected_cones
        )

        markers_msg = MarkerArray(markers=markers_list)

        self.detection_publisher.publish(detection_msg) # publish cone data
        self.marker_publisher.publish(markers_msg) # publish marker points data

        logger.info("Total Time:" + str(time.time()-start) + "\n")


## initialise ROS2 logging system
def init_logs() -> List[str]:
    args: List[str] = ['--ros-args']
    max_range: int = 17 #m

    path = str(pathlib.Path(__file__).parent.resolve())
    if not os.path.isdir(path + '/logs'):
        os.mkdir(path + '/logs')

    # defaults args
    print_mode = '--disable-stdout-logs'
    # processing args
    opts, arg = getopt.getopt(sys.argv[1:], str(), ['print', 'ros-args', 'range='])
    for opt, arg in opts:
        if opt == '--print':
            print_mode = '--enable-stdout-logs'
        elif opt == '--range':
            max_range = arg

    if not isinstance(max_range, int):
        raise ValueError('Invalid range: %s. Must be int' % max_range)

    args.append(print_mode)

    os.environ['ROS_LOG_DIR'] = f'{path}/logs/'
    os.environ.get('ROS_LOG_DIR')

    return args, max_range
    

def main(args=sys.argv[1:]):
    args, max_range = init_logs()

    # begin ros node
    rclpy.init(args=args)

    node = LidarProcessing(int(max_range))
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])
