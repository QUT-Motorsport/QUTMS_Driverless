# import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
# import ROS2 message libraries
from nav_msgs.msg import Odometry
# import custom message libraries
from driverless_msgs.msg import ConeDetectionStamped
from driverless_msgs.msg import Cone as QUTCone
from fs_msgs.msg import Track, Cone

# other python modules
from math import sin, cos
from typing import List
import sys
import os
import getopt
import logging
import datetime
import pathlib

from transforms3d.euler import quat2euler

# initialise logger
LOGGER = logging.getLogger(__name__)

class ConeLocator(Node):
    def __init__(self, max_range: float):
        super().__init__('map_processor')

        # sub to track for all cone locations relative to car start point
        self.create_subscription(Track, "/testing_only/track", self.map_callback, 10)
        # sub to odometry for car pose + velocity
        self.create_subscription(Odometry, "/testing_only/odom", self.odom_callback, 10)

        # publishes detected cones
        self.detection_publisher: Publisher = self.create_publisher(ConeDetectionStamped, "/detector/cone_detection", 1)

        self.max_range: float = max_range
        self.track: List[Cone] = []

        LOGGER.info('---Map processing node initialised---')


    def map_callback(self, track_msg: Track):
        LOGGER.info("Received map")
        # track cone list is taken as coords relative to the initial car position
        self.track = track_msg.track


    def odom_callback(self, odom_msg: Odometry):
        LOGGER.info("Received odom")

        w = odom_msg.pose.pose.orientation.w
        i = odom_msg.pose.pose.orientation.x
        j = odom_msg.pose.pose.orientation.y
        k = odom_msg.pose.pose.orientation.z

        # i, j, k angles in rad
        ai, aj, ak = quat2euler([w, i, j, k])

        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

        ref_cones: List[QUTCone] = []
        for cone in self.track:
            # displacement from car to cone
            x_dist = cone.location.x - x
            y_dist = cone.location.y - y

            ref_cone = QUTCone()
            # reference frame displacement with rotation 
            # uses k angle (z axis)
            ref_cone.location.x = x_dist*cos(ak) + y_dist*sin(ak)
            ref_cone.location.y = y_dist*cos(ak) - x_dist*sin(ak)
            ref_cone.location.z = 0.0
            ref_cone.color = cone.color

            if ref_cone.location.x > 0 and ref_cone.location.x < self.max_range \
                and ref_cone.location.y > -9 and ref_cone.location.y < 9:
                ref_cones.append(ref_cone)

        # create message with cones
        detection_msg = ConeDetectionStamped(
            header=odom_msg.header,
            cones=ref_cones
        )
        self.detection_publisher.publish(detection_msg) # publish cone data


def main(args=sys.argv[1:]):
    # defaults args
    loglevel = 'info'
    print_logs = False
    max_range = 20 #m

    # processing args
    # opts, arg = getopt.getopt(args, str(), ['log=', 'print_logs', 'range='])

    # # TODO: provide documentation for different options
    # for opt, arg in opts:
    #     if opt == '--log':
    #         loglevel = arg
    #     elif opt == '--print_logs':
    #         print_logs = True
    #     elif opt == '--range':
    #         max_range = arg
    # # validating args
    numeric_level = getattr(logging, loglevel.upper(), None)
    # if not isinstance(numeric_level, int):
    #     raise ValueError('Invalid log level: %s' % loglevel)
    # if not isinstance(max_range, int):
    #     raise ValueError('Invalid range: %s. Must be int' % max_range)

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
