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
import time
from typing import List
import sys
import os
import getopt
import logging
import datetime
import pathlib

# import ROS function that has been ported to ROS2 by
# SebastianGrans https://github.com/SebastianGrans/ROS2-Point-Cloud-Demo
from .scripts.read_pcl import read_points_list
# lidar cone detection algorithm
from .scripts.ground_plane_estimation import lidar_main, lidar_init


LOGGER = logging.getLogger(__name__)


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


class LidarProcessing(Node):
    def __init__(self, pc2_topic: str, visualise: bool, display: bool, max_range: int):
        super().__init__('lidar_processor')

        self.pcl_subscription = self.create_subscription(
            PointCloud2,
            pc2_topic,
            self.callback,
            10)

        lidar_init(visualise, display, "/home/developer/datasets/figures/", max_range)

        self.detection_publisher: Publisher = self.create_publisher(
            ConeDetectionStamped, 
            "lidar/cone_detection", 
            1)

        self.marker_publisher: Publisher = self.create_publisher(
            MarkerArray, 
            "lidar/debug_cones_array", 
            1)

        LOGGER.info('---LiDAR processing node initialised---')


    def callback(self, pc2_msg: PointCloud2):
        """ 
        lidar point cloud message sent here. 
        used to call funtions to find cone coords.
        to get the xyz location and colour of cones.
        """
        
        start: float = time.time()
        # Convert the list of floats into a list of xyz coordinates

        point_array: List(List) = read_points_list(pc2_msg)

        LOGGER.info("\nRead Time:" + str(time.time()-start))

        # calls main module from ground estimation algorithm
        cones: List[List] = lidar_main(point_array) 

        LOGGER.info("\nDetected cones:" + str(len(cones)))
        
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
                pc2_msg.header,
            ))

        detection_msg = ConeDetectionStamped(
            header=pc2_msg.header,
            cones=detected_cones
        )

        markers_msg = MarkerArray(markers=markers_list)

        self.detection_publisher.publish(detection_msg) # publish cone data
        self.marker_publisher.publish(markers_msg) # publish marker points data

        LOGGER.info("Total Time:" + str(time.time()-start))


def main(args=sys.argv[1:]):
    # defaults args
    pc2_topic = '/velodyne_points'
    loglevel = 'info'
    print_logs = False
    display = False
    visualise = False
    max_range = 20 #m

    # processing args
    opts, arg = getopt.getopt(args, str(), ['sim', 'log=', 'print_logs', 'display', 'visualise', 'range='])

    # TODO: provide documentation for different options
    for opt, arg in opts:
        if opt == '--sim':
            pc2_topic = "/lidar/Lidar2"
        elif opt == '--log':
            loglevel = arg
        elif opt == '--print_logs':
            print_logs = True
        elif opt == '--display':
            display = True
        elif opt == '--visualise':
            visualise = True
        elif opt == '--range':
            max_range = arg

    # validating args
    numeric_level = getattr(logging, loglevel.upper(), None)

    if not isinstance(numeric_level, int):
        raise ValueError('Invalid log level: %s' % loglevel)

    if not isinstance(max_range, int):
        raise ValueError('Invalid range: %s. Must be int' % max_range)

    # setting up logging
    path = str(pathlib.Path(__file__).parent.resolve())
    if not os.path.isdir(path + '/logs'):
        os.mkdir(path + '/logs')

    date = datetime.datetime.now().strftime('%d_%m_%Y_%H_%M_%S')
    logging.basicConfig(
        filename=f'{path}/logs/{date}.log',
        filemode='w',
        format='%(asctime)s | %(levelname)s:%(name)s: %(message)s',
        datefmt='%I:%M:%S %p',
        # encoding='utf-8',
        level=numeric_level,
    )

    # terminal stream
    if print_logs:
        stdout_handler = logging.StreamHandler(sys.stdout)
        LOGGER.addHandler(stdout_handler)

    LOGGER.info(f'args = {args}')
    LOGGER.info(f'pc_node = {pc2_topic}')

    # begin ros node
    rclpy.init(args=args)

    node = LidarProcessing(pc2_topic, display, visualise, int(max_range))
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])
