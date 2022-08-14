import sys
import logging

import rclpy

from .utils import Config
from .cone_detection_node import ConeDetectionNode


def local_data_stream():
    raise NotImplementedError()


def real_time_stream(args: list, config: Config) -> None:
    # Init node
    rclpy.init(args=args)
    cone_detection_node: ConeDetectionNode = ConeDetectionNode(config)
    rclpy.spin(cone_detection_node)
    
    # Destroy node explicitly
    cone_detection_node.destroy_node()
    rclpy.shutdown()


def main(args=sys.argv[1:]):
    # Init config
    config: Config = Config()
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
