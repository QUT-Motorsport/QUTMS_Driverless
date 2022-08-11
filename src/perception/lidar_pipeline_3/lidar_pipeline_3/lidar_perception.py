import sys
import logging
LOGGER = logging.getLogger(__name__)

import rclpy

from .config import Config
from .ConeDetectionNode import ConeDetectionNode


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

    # Init logging
    logging.basicConfig(filename=f'{config.runtime_dir}/output.log',
                        filemode='w',
                        format='%(asctime)s,%(msecs)03d | %(levelname)s | %(filename)s %(lineno)s: %(message)s',
                        datefmt='%H:%M:%S',
                        # encoding='utf-8',
                        level=config.numeric_loglevel)
    
    if not config.print_logs:
        print("WARNING: --print_logs flag not specified")
        print("Running lidar_perception without printing to terminal ...")
    else:
        stdout_handler = logging.StreamHandler(sys.stdout)
        LOGGER.addHandler(stdout_handler)
    
    LOGGER.info(f'initialised at {config.datetimestamp}')
    LOGGER.info(f'args = {args}')
    
    print(__name__)
    
    # Init data stream
    if config.data_path:
        # Utilise local data source
        raise NotImplementedError()
    else:
        # Utilise real-time source
        real_time_stream(args, config)

if __name__ == '__main__':
    main(sys.argv[1:])
