# Import ROS2 Modules
import rclpy
from rclpy.node import Node

# Import ROS2 Message Modules
from sensor_msgs.msg import PointCloud2

# Import Modules
import datetime
import pathlib
import os

# Import Logging
import logging
LOGGER = logging.getLogger(__name__)

LIDAR_NODE = '/velodyne_points'


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
            String,
            'cone_sensing/cones',
            10)

        self.marker_publisher = self.create_publisher(
            String,
            'cone_sensing/cone_markers',
            10)

        self.count = 0

    def pc_callback(self, pc_msg: PointCloud2):
        pass


def main(args=None):
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
                        level=logging.INFO)

    LOGGER.info('Hi from lidar_pipeline_2.')

    # Publisher Node
    rclpy.init(args=args)

    cone_sensing_node = ConeSensingNode()

    rclpy.spin(cone_sensing_node)

    # Destroy the node explicitly
    cone_sensing_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
