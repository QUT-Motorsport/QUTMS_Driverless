# import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
# import ROS2 message libraries
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration, Time
from nav_msgs.msg import Odometry
# import custom message libraries
from driverless_msgs.msg import ConeDetectionStamped
from driverless_msgs.msg import Cone as QUTCone
from fs_msgs.msg import Track, Cone

from transforms3d.euler import quat2euler

# other python modules
import time
import math
from typing import List
import sys
import os
import getopt
import logging
import datetime
import pathlib


LOGGER = logging.getLogger(__name__)


def marker_msg(
    colour: int, 
    x_coord: float, 
    y_coord: float, 
    ID: int, 
    header: Header,
    quaternion = None
) -> Marker: 
    """
    Creates a Marker object for cones or a car.
    * param colour: Cone.COLOR
    * param x_coord: x position relative to parent frame
    * param y_coord: y position relative to parent frame
    * param ID: Unique for markers in the same frame
    * param header: passed in because creating time is dumb
    * param quaternion: for car orientation
    * return: Marker
    """

    marker = Marker()
    marker.header = header
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

    marker.color.a = 1.0 # alpha

    marker.lifetime = Duration(sec=10, nanosec=100000)

    if colour == 0: # blue cone
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
    elif colour == 1: # yellow cone
        marker.color.r = 0.901
        marker.color.g = 0.858
        marker.color.b = 0.039
    elif colour == 2: # orange cone
        marker.color.r = 0.901
        marker.color.g = 0.309
        marker.color.b = 0.039
    else: # not a cone colour (this is a car)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        # approx car size
        marker.scale.x = 1.8
        marker.scale.y = 1.0
        marker.scale.z = 0.6
        # use orientation passed
        marker.pose.orientation = quaternion

        marker.ns = "car"
        marker.type = Marker.CUBE
        # update as fast as odom topic is subbed
        marker.lifetime = Duration(sec=0, nanosec=4200)

    return marker


class ConeLocator(Node):
    def __init__(self, max_range: float):
        super().__init__('map_processor')

        # sub to track for all cone locations relative to car start point
        self.track_subscription = self.create_subscription(
            Track,
            "/testing_only/track",
            self.map_callback,
            10)
        # sub to odometry for car pose + velocity
        self.odom_subscription = self.create_subscription(
            Odometry,
            "/testing_only/odom",
            self.odom_callback,
            10)

        # publishes detected cones
        self.detection_publisher: Publisher = self.create_publisher(
            ConeDetectionStamped, 
            "zed_detector/cone_detection", 
            1)
        # publishes rviz cone markers
        self.marker_publisher: Publisher = self.create_publisher(
            MarkerArray, 
            "view/debug_cones_array", 
            1)
        # publishes rviz car marker
        self.car_publisher: Publisher = self.create_publisher(
            Marker, 
            "view/debug_car", 
            1)

        LOGGER.info('---Map processing node initialised---')

        self.max_range: float = max_range
        self.track: List[Cone] = []
        self.odom_header = Header()


    def map_callback(self, track_msg: Track):
        logger = self.get_logger()
        logger.info("Received map")
        
        self.track = track_msg.track

        markers_list: List[Marker] = []
        for i, cone in enumerate(track_msg.track):
            # add on each cone to published array
            markers_list.append(marker_msg(
                cone.color,
                cone.location.x, 
                cone.location.y, 
                i, 
                self.odom_header,
            ))

        markers_msg = MarkerArray(markers=markers_list)
        self.marker_publisher.publish(markers_msg) # publish marker points data


    def odom_callback(self, odom_msg: Odometry):
        logger = self.get_logger()
        # logger.info("Received odom")

        self.odom_header = odom_msg.header

        w = odom_msg.pose.pose.orientation.w
        i = odom_msg.pose.pose.orientation.x
        j = odom_msg.pose.pose.orientation.y
        k = odom_msg.pose.pose.orientation.z

        ai, aj, ak = quat2euler([w, i, j, k])

        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

        ref_cones: List[QUTCone] = []
        for cone in self.track:
            x_dist = cone.location.x - x
            y_dist = cone.location.y - y


            ref_cone = QUTCone()
            ref_cone.location.x = x_dist*math.cos(ak) + y_dist*math.sin(ak)
            ref_cone.location.y = y_dist*math.cos(ak) - x_dist*math.sin(ak)
            ref_cone.location.z = 0.0
            ref_cone.color = cone.color

            if ref_cone.location.x > 0 and ref_cone.location.x < self.max_range \
                and ref_cone.location.y > -10 and ref_cone.location.y < 10:
                ref_cones.append(ref_cone)

        detection_msg = ConeDetectionStamped(
            header=odom_msg.header,
            cones=ref_cones
        )
        self.detection_publisher.publish(detection_msg) # publish cone data

        car_marker = marker_msg(
                3,
                x, 
                y, 
                0, 
                odom_msg.header,
                odom_msg.pose.pose.orientation,
            )
        self.car_publisher.publish(car_marker) # publish marker points data


def main(args=sys.argv[1:]):
    # defaults args
    loglevel = 'info'
    print_logs = False
    max_range = 25 #m

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
