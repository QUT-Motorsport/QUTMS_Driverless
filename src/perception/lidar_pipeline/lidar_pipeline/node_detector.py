import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from driverless_msgs.msg import Cone, ConeDetectionStamped
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField

from .process import *


def fields_to_dtype(fields, point_step):
    """
    FROM ROS2_NUMPY
    Convert a list of PointFields to a numpy record datatype.
    """
    DUMMY_FIELD_PREFIX = "__"
    # mappings between PointField types and numpy types
    type_mappings = [
        (PointField.INT8, np.dtype("int8")),
        (PointField.UINT8, np.dtype("uint8")),
        (PointField.INT16, np.dtype("int16")),
        (PointField.UINT16, np.dtype("uint16")),
        (PointField.INT32, np.dtype("int32")),
        (PointField.UINT32, np.dtype("uint32")),
        (PointField.FLOAT32, np.dtype("float32")),
        (PointField.FLOAT64, np.dtype("float64")),
    ]
    pftype_to_nptype = dict(type_mappings)

    offset = 0
    np_dtype_list = []
    for f in fields:
        while offset < f.offset:
            # might be extra padding between fields
            np_dtype_list.append(("%s%d" % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        dtype = pftype_to_nptype[f.datatype]
        if f.count != 1:
            dtype = np.dtype((dtype, f.count))

        np_dtype_list.append((f.name, dtype))
        offset += pftype_to_nptype[f.datatype].itemsize * f.count

    # might be extra padding between points
    while offset < point_step:
        np_dtype_list.append(("%s%d" % (DUMMY_FIELD_PREFIX, offset), np.uint8))
        offset += 1

    return np_dtype_list


def cone_msg(x: float, y: float) -> Cone:
    """
    Create a Cone message from x and y coordinates.

    Args:
        x (float): The x coordinate of the cone (LiDAR sensor is origin).
        y (float): The y coordinate of the cone (LiDAR sensor is origin).

    Returns:
        Cone: The cone message.
    """
    location: Point = Point(x=x, y=y, z=LIDAR_HEIGHT_ABOVE_GROUND)

    # LiDAR Pipeline does not identify cone colour
    return Cone(location=location, color=Cone.UNKNOWN)


class LiDARDetectorNode(Node):
    """
    Node for detecting cones and their locations from a point cloud.

    Args:
        Node (rclpy.node.Node): The ROS2 node class.
        _config (Config): Configuration object
    """

    def __init__(self) -> None:
        super().__init__("lidar_detector_node")

        # declare parameters
        LOG_LEVEL: str = self.declare_parameter("log_level", "INFO").value

        # convert log level string to ros2 logging level
        log_level: int = getattr(rclpy.impl.logging_severity.LoggingSeverity, LOG_LEVEL.upper())
        self.get_logger().set_level(log_level)

        # Init variables
        self.iteration: int = 0
        self.average_runtime: float = 0

        # Create subscribers and publishers
        self.pointcloud_sub: Subscription = self.create_subscription(PointCloud2, "/velodyne_points", self.callback, 1)
        self.detection_publisher: Publisher = self.create_publisher(ConeDetectionStamped, "/lidar/cone_detection", 1)

        # Log info
        self.get_logger().info("---LiDAR detector node initialised---")

    def callback(self, msg: PointCloud2) -> None:
        """
        Callback function for when point cloud data is received.

        Args:
            point_cloud_msg (PointCloud2): The point cloud message.
        """
        self.iteration += 1

        # Convert PointCloud2 message from LiDAR sensor to numpy array
        start_time: float = time.perf_counter()
        dtype_list: list = fields_to_dtype(msg.fields, msg.point_step)  # x, y, z, intensity, ring
        point_cloud: np.ndarray = np.frombuffer(msg.data, dtype_list)

        # Remove points behind car
        point_cloud = point_cloud[point_cloud["x"] > 0]

        # Remove points that are above the height of cones
        point_cloud = point_cloud[point_cloud["z"] < LHAG_ERR * (LIDAR_HEIGHT_ABOVE_GROUND + CONE_HEIGHT)]

        # Compute point normals
        point_norms = np.linalg.norm([point_cloud["x"], point_cloud["y"]], axis=0)

        # Remove points that are outside of range or have a norm of 0
        mask = point_norms <= LIDAR_RANGE  # & (point_norms != 0)
        point_norms = point_norms[mask]
        point_cloud = point_cloud[mask]

        # Get segments and prototype points
        segments, bins = get_discretised_positions(point_cloud["x"], point_cloud["y"], point_norms)
        proto_segs_arr, proto_segs, seg_bin_z_ind = get_prototype_points(point_cloud["z"], segments, bins, point_norms)

        # Extract ground plane
        ground_plane = get_ground_plane_single_core(proto_segs_arr, proto_segs)

        # Label points
        point_labels, ground_lines_arr = label_points(point_cloud["z"], segments, bins, seg_bin_z_ind, ground_plane)
        object_points = point_cloud[point_labels]

        if object_points.size == 0:
            self.get_logger().info("No objects points detected")
            return []

        object_centers, objects = group_points(object_points)

        ground_points = point_cloud[~point_labels]
        obj_segs, obj_bins, reconstructed_objects, reconstructed_centers, avg_object_intensity = reconstruct_objects(
            ground_points, segments[~point_labels], bins[~point_labels], object_centers, objects
        )

        cone_locations, cone_points, cone_intensities = cone_filter(
            segments,
            bins,
            ground_lines_arr,
            obj_segs,
            obj_bins,
            object_centers,
            reconstructed_objects,
            reconstructed_centers,
            avg_object_intensity,
        )

        # Calculate runtime statistics
        duration: float = time.perf_counter() - start_time
        self.average_runtime = (self.average_runtime * (self.iteration - 1) + duration) / self.iteration
        self.get_logger().debug(
            f"Current Hz: {round(1 / duration, 2)}\t| Average Hz: {round(1 / self.average_runtime, 2)}"
        )

        # If no cones were detected, return
        if len(cone_locations) == 0:
            return

        # Convert cone locations to ConeDetection messages and publish
        detected_cones: list = [cone_msg(cone[0], cone[1]) for cone in cone_locations]
        detection_msg = ConeDetectionStamped(header=msg.header, cones=detected_cones)
        self.detection_publisher.publish(detection_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LiDARDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
