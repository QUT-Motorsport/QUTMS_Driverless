# Import ROS2 Modules
import rclpy
from rclpy.node import Node

# Import ROS2 Message Modules
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray

# Import Custom Message Modules
from driverless_msgs.msg import LidarCones

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
    def __init__(self):
        super().__init__('cone_sensing')

        LOGGER.info(f'LIDAR_NODE = {LIDAR_NODE}')

        self.pc_subscription = self.create_subscription(
            PointCloud2,
            LIDAR_NODE,
            self.pc_callback,
            2)
        self.pc_subscription  # Prevent unused variable warning

        self.cone_publisher = self.create_publisher(
            LidarCones,
            'cone_sensing/cones',
            5)

        self.marker_publisher = self.create_publisher(
            MarkerArray,
            'cone_sensing/cone_markers',
            5)

        self.count = 0

    def pc_callback(self, pc_msg):
        pass


def main(args=None):
    # Defaults
    pc_node = '/velodyne_points'
    loglevel = 'info'

    # Processing args
    opts, arg = getopt.getopt(args, str(), ['--pc_node=', '--log='])

    for opt, arg in opts:
        if opt == '--pc_node':
            pc_node = arg
        elif opt == '--log':
            loglevel = arg

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

    LOGGER.info('Hi from lidar_pipeline_2.')

    # Setting up node
    rclpy.init(args=args)

    cone_sensing_node = ConeSensingNode()

    rclpy.spin(cone_sensing_node)

    # Destroy the node explicitly
    cone_sensing_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])
