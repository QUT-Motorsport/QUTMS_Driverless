import sys
import logging
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from sensor_msgs.msg import PointCloud2
import ros2_numpy as rnp

from . import utils
from . import constants as const
from driverless_msgs.msg import ConeDetectionStamped

# For typing
from .utils import Config

class ConeDetectionNode(Node):
    def __init__(self, _config: Config) -> None:
        super().__init__("cone_detection")
        
        self.iteration: int = 0
        self.average_duration: float = 0

        self.config: Config = _config
        self.pc_subscription: Subscription = self.create_subscription(PointCloud2, self.config.pc_node, self.pc_callback, 1)
        self.cone_publisher: Publisher = self.create_publisher(ConeDetectionStamped, "lidar/cone_detection", 1)
        
        self.config.logger.info("Waiting for point cloud data...")
    
    def pc_callback(self, pc_msg: PointCloud2) -> None:
        self.iteration += 1
        
        # Convert PointCloud2 message from LiDAR sensor to numpy array
        start_time = time.perf_counter()
        dtype_list = rnp.point_cloud2.fields_to_dtype(pc_msg.fields, pc_msg.point_step)  # x y z intensity ring
        point_cloud = np.frombuffer(pc_msg.data, dtype_list)

        # Compute point normals
        point_norms = np.linalg.norm([point_cloud["x"], point_cloud["y"]], axis=0)
        
        # Remove points that are outside of range, have a norm of 0, or are behind the car 
        mask = (point_norms <= const.LIDAR_RANGE) & (point_norms != 0) # & (point_cloud['x'] < 0)
        point_norms = point_norms[mask]
        point_cloud = point_cloud[mask]
        
        # Number of points in point_cloud
        point_count = point_cloud.shape[0]
        self.config.logger.info(f"{point_count} points remain after processing point cloud")

        duration = time.perf_counter() - start_time
        self.average_duration = (self.average_duration * (self.iteration - 1) + duration) / self.iteration
        self.config.logger.info(f"Current Hz: {round(1 / duration, 2)}\t| Average Hz: {round(1 / self.average_duration, 2)}")


def real_time_stream(args: list, config: Config) -> None:
    # Init node
    rclpy.init(args=args)
    cone_detection_node: ConeDetectionNode = ConeDetectionNode(config)
    rclpy.spin(cone_detection_node)
    
    # Destroy node explicitly
    cone_detection_node.destroy_node()
    rclpy.shutdown()


def local_data_stream():
    raise NotImplementedError()


def main(args=sys.argv[1:]):
    # Init config
    config: Config = utils.Config()
    config.update(args)
    
    # Check if logs should be printed 
    if not config.print_logs:
        print("WARNING: --print_logs flag not specified")
        print("Running lidar_perception without printing to terminal...")
    else:
        stdout_handler = logging.StreamHandler(sys.stdout)
        config.logger.addHandler(stdout_handler)
    
    # Log time and arguments provided
    config.logger.info(f'initialised at {config.datetimestamp}')
    config.logger.info(f'args = {args}')
    
    # Init data stream
    if config.data_path:
        # Utilise local data source
        local_data_stream()
    else:
        # Utilise real-time source
        real_time_stream(args, config)

if __name__ == '__main__':
    main(sys.argv[1:])
