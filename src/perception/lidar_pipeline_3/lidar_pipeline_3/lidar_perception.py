import logging
import numpy as np
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
import ros2_numpy as rnp

from driverless_msgs.msg import Cone, ConeDetectionStamped
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2

from . import constants as const
from .library import lidar_manager
from . import utils
from . import video_stitcher

# For typing
from .utils import Config
from logging import StreamHandler


def cone_msg(x: float, y: float) -> Cone:
    """Create a Cone message from x and y coordinates.

    Args:
        x (float): The x coordinate of the cone (LiDAR sensor is origin).
        y (float): The y coordinate of the cone (LiDAR sensor is origin).

    Returns:
        Cone: The cone message.
    """
    location: Point = Point(x=x, y=y, z=const.LIDAR_HEIGHT_ABOVE_GROUND)

    # LiDAR Pipeline does not identify cone colour
    return Cone(location=location, color=Cone.UNKNOWN)


class ConeDetectionNode(Node):
    """Node for detecting cones and their locations from a point cloud.

    Args:
        Node (rclpy.node.Node): The ROS2 node class.
    """
    def __init__(self, _config: Config) -> None:
        super().__init__("lidar_processor_node")

        self.iteration: int = 0
        self.average_runtime: float = 0

        self.config: Config = _config
        self.pc_subscription: Subscription = self.create_subscription(
            PointCloud2, self.config.pc_node, self.pc_callback, 1
        )
        self.cone_publisher: Publisher = self.create_publisher(ConeDetectionStamped, "/lidar/cone_detection", 1)

        self.config.logger.info("Waiting for point cloud data...")

    def pc_callback(self, point_cloud_msg: PointCloud2) -> None:
        """Callback function for when point cloud data is received.

        Args:
            point_cloud_msg (PointCloud2): The point cloud message.
        """
        self.iteration += 1

        # Convert PointCloud2 message from LiDAR sensor to numpy array
        start_time: float = time.perf_counter()
        dtype_list: list = rnp.point_cloud2.fields_to_dtype(
            point_cloud_msg.fields, point_cloud_msg.point_step
        )  # x y z intensity ring
        point_cloud: np.ndarray = np.frombuffer(point_cloud_msg.data, dtype_list)

        cone_locations: np.ndarray = lidar_manager.locate_cones(self.config, point_cloud, start_time)

        duration: float = time.perf_counter() - start_time
        self.average_runtime = (self.average_runtime * (self.iteration - 1) + duration) / self.iteration
        self.config.logger.info(
            f"Current Hz: {round(1 / duration, 2)}\t| Average Hz: {round(1 / self.average_runtime, 2)}"
        )

        if len(cone_locations) == 0:
            return

        detected_cones: list = [cone_msg(cone[0], cone[1]) for cone in cone_locations]

        detection_msg: ConeDetectionStamped = ConeDetectionStamped(header=point_cloud_msg.header, cones=detected_cones)
        self.cone_publisher.publish(detection_msg)


def real_time_stream(args: list, config: Config) -> None:
    """Run the node for real-time data.

    Args:
        args (list): The arguments provided to the node.
        config (Config): The configuration object.
    """
    # Init node
    rclpy.init(args=args)
    cone_detection_node: ConeDetectionNode = ConeDetectionNode(config)
    rclpy.spin(cone_detection_node)

    # Destroy node explicitly
    cone_detection_node.destroy_node()
    rclpy.shutdown()


def local_data_stream():
    """Run the pipeline using exported point cloud frames instead of a ROS bag.
    This is useful for debugging and testing (e.g., IDE breakpoints can be used).
    """
    raise NotImplementedError()


def main(args=sys.argv[1:]):
    # Init config
    config: Config = utils.Config()
    config.update(args)

    # Check if logs should be printed
    if not config.print_logs:
        print("WARNING: --print_logs flag not specified")
        print("Running lidar_perception without printing to terminal...\n")
    else:
        stdout_handler: StreamHandler = logging.StreamHandler(sys.stdout)
        config.logger.addHandler(stdout_handler)
    
    # Figures warning
    if config.show_figures:
        print("WARNING: --show_figures flag specified")
        print(
            "The QUTMS watermark will overlap matplotlib interactive figures." +
            "\nHowever, it will be formatted correctly on the saved figures in the output direcrory...\n"
        )

    # Log time and arguments provided
    config.logger.info(f"initialised at {config.datetimestamp}")
    config.logger.info(f"args = {args}")

    # Init data stream
    # Check to see if video should be stitched instead of running pipeline
    if config.video_from_session:
        video_stitcher.stitch_figures(
            f"{const.OUTPUT_DIR}/{config.video_from_session}{const.FIGURES_DIR}",
            f"{const.OUTPUT_DIR}/{config.video_from_session}_{const.VIDEOS_DIR}/{const.VIDEOS_DIR}",
        )
    elif config.data_path:
        # Use local data source
        local_data_stream()
    else:
        # Use real-time source
        real_time_stream(args, config)


if __name__ == "__main__":
    main(sys.argv[1:])
