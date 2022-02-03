# import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
# import ROS2 message libraries
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration, Time
# import custom message libraries
from driverless_msgs.msg import ConeDetectionStamped
from fs_msgs.msg import Track, Cone

# other python modules
import time
from typing import List
import sys
import os
import getopt
import logging
import datetime
import pathlib


LOGGER = logging.getLogger(__name__)


def marker_msg(x_coord: float, y_coord: float, ID: int, stamp: Time) -> Marker: 
    marker = Marker()
    marker.header.frame_id = 'fsds/map'
    marker.header.stamp = stamp
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



class ConeLocator(Node):
    def __init__(self, max_range: float):
        super().__init__('map_processor')

        self.track_subscription = self.create_subscription(
            Track,
            "/testing_only/track",
            self.map_callback,
            10)

        self.odom_subscription = self.create_subscription(
            Track,
            "/testing_only/odom",
            self.map_callback,
            10)

        self.detection_publisher: Publisher = self.create_publisher(
            ConeDetectionStamped, 
            "view/cone_detection", 
            1)
        self.marker_publisher: Publisher = self.create_publisher(
            MarkerArray, 
            "view/debug_cones_array", 
            1)

        LOGGER.info('---Map processing node initialised---')

        self.max_range: float = max_range
        self.track: List[Cone] = []


    def map_callback(self, track_msg: Track):
        logger = self.get_logger()
        logger.info("Received map")
        
        self.track = track_msg.track

        markers_list: List[Marker] = []
        for i, cone in enumerate(track_msg.track):

            marker = Marker()
            marker.header.frame_id = 'fsds/map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "current_scan"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = cone.location.x
            marker.pose.position.y = cone.location.y
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            # scale out of 1x1x1m
            marker.scale.x = 0.228
            marker.scale.y = 0.228
            marker.scale.z = 0.325

            if cone.color == 0:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            elif cone.color == 1:
                marker.color.r = 0.901
                marker.color.g = 0.858
                marker.color.b = 0.039
            elif cone.color == 2:
                marker.color.r = 0.901
                marker.color.g = 0.309
                marker.color.b = 0.039
            
            marker.color.a = 1.0


            marker.lifetime = Duration(sec=10, nanosec=0)

            markers_list.append(marker)

        markers_msg = MarkerArray(markers=markers_list)
        self.marker_publisher.publish(markers_msg) # publish marker points data


def main(args=sys.argv[1:]):
    # defaults args
    loglevel = 'info'
    print_logs = False
    max_range = 15 #m

    # processing args
    opts, arg = getopt.getopt(args, str(), ['log=', 'print_logs', 'range='])

    # TODO: provide documentation for different options
    for opt, arg in opts:
        if opt == '--log':
            loglevel = arg
        elif opt == '--print_logs':
            print_logs = True
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

    # begin ros node
    rclpy.init(args=args)

    node = ConeLocator(int(max_range))
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])
