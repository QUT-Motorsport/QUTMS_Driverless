# Import Logging
import logging
LOGGER = logging.getLogger(__name__)

# Import Modules
import datetime
import pathlib
import test

# Import ROS2 Modules
import rclpy
from rclpy.node import Node

class ConePerceptionNode(Node):
    def __init__(self):
        super().__init__("cone_perception")
        pass


def main(args=None):
    # Setting up logging
    date = datetime.datetime.now().strftime('%d_%m_%Y_%H_%M_%S')
    path = pathlib.Path(__file__).parent.resolve()
    logging.basicConfig(filename=f'{path}/logs/{date}.log', filemode='w', format='%(asctime)s | %(levelname)s:%(name)s: %(message)s', datefmt='%I:%M:%S %p', encoding='utf-8', level=logging.INFO)
    
    LOGGER.info('Hi from lidar_pipeline_2.')

    rclpy.init(args=args)
    cone_perception_node = ConePerceptionNode()

    rclpy.spin(cone_perception_node)

    rclpy.shutdown()
    
    test.main()


if __name__ == '__main__':
    main()
