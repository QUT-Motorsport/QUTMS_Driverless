import utils

from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from sensor_msgs.msg import PointCloud2

from driverless_msgs.msg import ConeDetectionStamped

from .utils import Config # For typing


class ConeDetectionNode(Node):
    def __init__(self, _config: Config) -> None:
        super().__init__("cone_detection")

        self.config: Config = _config
        self.pc_subscription: Subscription = self.create_subscription(PointCloud2, self.config.pc_node, self.pc_callback, 1)
        self.cone_publisher: Publisher = self.create_publisher(ConeDetectionStamped, "lidar/cone_detection", 1)
        
        self.config.logger.info("Waiting for point cloud data...")
    
    def pc_callback(self, pc_msg: PointCloud2) -> None:
        timestamp: str = utils.get_timestamp()
        pass