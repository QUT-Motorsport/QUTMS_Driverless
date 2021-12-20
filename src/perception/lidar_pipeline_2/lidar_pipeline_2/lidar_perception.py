# Import ROS2 Modules
import rclpy
from rclpy.node import Node

# Import ROS2 Message Modules
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray

# Import ROS2 Helper Modules
from .library import ros2_numpy as rnp

# Import Custom Message Modules
from driverless_msgs.msg import ConeDetectionStamped

# Import Python Modules
import datetime
import pathlib
import os
import getopt
import sys

# Import Logging
import logging
LOGGER = logging.getLogger(__name__)


class ConeSensingNode(Node):
    def __init__(self, pc_node):
        super().__init__('cone_sensing')

        self.pc_subscription = self.create_subscription(
            PointCloud2,
            pc_node,
            self.pc_callback,
            2)
        self.pc_subscription  # Prevent unused variable warning

        self.cone_publisher = self.create_publisher(
            ConeDetectionStamped,
            'cone_sensing/cones',
            5)

        self.marker_publisher = self.create_publisher(
            MarkerArray,
            'cone_sensing/cone_markers',
            5)

        self.count = 0

    def pc_callback(self, pc_msg):
        pc_matrix = pcl_helper.pointcloud2_to_array(pc_msg)
        LOGGER.debug(pc_matrix)
        pass


def main(args=None):
    # Defaults
    pc_node = '/velodyne_points'
    loglevel = 'info'
    print_logs = False

    # Processing args
    opts, arg = getopt.getopt(args, str(), ['--pc_node=', '--log=', '--print_logs'])

    for opt, arg in opts:
        if opt == '--pc_node':
            pc_node = arg
        elif opt == '--log':
            loglevel = arg
        elif opt == '--print_logs':
            print_logs = True

    # Validating args
    numeric_level = getattr(logging, loglevel.upper(), None)

    if not isinstance(numeric_level, int):
        raise ValueError('Invalid log level: %s' % loglevel)

    # Setting up logging
    path = str(pathlib.Path(__file__).parent.resolve())
    if not os.path.isdir(path + '/logs'):
        os.mkdir(path + '/logs')

    date = datetime.datetime.now().strftime('%d_%m_%Y_%H_%M_%S')
    logging.basicConfig(filename=f'{path}/logs/{date}.log',
                        filemode='w',
                        format='%(asctime)s | %(levelname)s:%(name)s: %(message)s',
                        datefmt='%I:%M:%S %p',
                        encoding='utf-8',
                        level=logging.numeric_level)

    # Printing logs to terminal
    if print_logs:
        stdout_handler = logging.StreamHandler(sys.stdout)
        LOGGER.addHandler(stdout_handler)

    LOGGER.info('Hi from lidar_pipeline_2.')
    LOGGER.info(f'args = {args}')
    LOGGER.info(f'pc_node = {pc_node}')

    # Setting up node
    rclpy.init(args=args)

    cone_sensing_node = ConeSensingNode(pc_node)

    rclpy.spin(cone_sensing_node)

    # Destroy the node explicitly
    cone_sensing_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])
