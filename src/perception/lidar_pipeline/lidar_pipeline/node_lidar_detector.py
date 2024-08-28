import array
import array
import math
import time

import numpy as np
from sklearn.cluster import DBSCAN

import rclpy
from rclpy.node import Node

from driverless_msgs.msg import Cone, ConeDetectionStamped
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField

from driverless_common.common import QOS_LATEST, FPSHandler

DUMMY_FIELD_PREFIX = "__"

def array_to_pointcloud2(point_cloud_msg, cloud_arr, dtype_list):
    # The PointCloud2.data setter will create an array.array object for you if you don't
    # provide it one directly. This causes very slow performance because it iterates
    # over each byte in python.
    # Here we create an array.array object using a memoryview, limiting copying and
    # increasing performance.

    # remove the dummy fields that were added
    cloud_arr = cloud_arr[
        [fname for fname, _type in dtype_list if not (fname[: len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]
    ]

    point_cloud_msg.height = 1
    point_cloud_msg.width = cloud_arr.shape[0]
    point_cloud_msg.row_step = point_cloud_msg.point_step * point_cloud_msg.width

    # make it 2d (even if height will be 1)
    cloud_arr = np.atleast_2d(cloud_arr)

    memory_view = memoryview(cloud_arr)
    if memory_view.nbytes > 0:
        array_bytes = memory_view.cast("B")
    else:
        # Casting raises a TypeError if the array has no elements
        array_bytes = b""
    as_array = array.array("B")
    as_array.frombytes(array_bytes)
    point_cloud_msg.data = as_array
    return point_cloud_msg


def fields_to_dtype(fields, point_step):
    """
    FROM ROS2_NUMPY
    Convert a list of PointFields to a numpy record datatype.
    """
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
    location: Point = Point(x=x, y=y, z=-0.15)

    # LiDAR Pipeline does not identify cone colour
    return Cone(location=location, color=Cone.UNKNOWN)


class LiDARDetectorNode(Node):

    fps = FPSHandler()

    def __init__(self):
        super().__init__("lidar_detector_node")

        # Initialise parameters
        self.initialise_params()

        # Create subscribers and publishers
        self.create_subscription(PointCloud2, "/lidar/objects", self.callback, QOS_LATEST)

        self.detection_publisher = self.create_publisher(ConeDetectionStamped, "/lidar/cone_detection", 1)
        self.point_cloud_publisher_debug = self.create_publisher(PointCloud2, "/lidar/cone_points", QOS_LATEST)

        # Log info
        self.get_logger().info("---LiDAR detector node initialised---")

    def initialise_params(self):
        # declare parameters

        # "t_" prefix is used to denote thresholds

        self.declare_parameter("log_level", "DEBUG")
        # Max range of points to process (metres)
        self.declare_parameter("lidar_range", 30)
        self.declare_parameter("min_range", 2)

        # LiDAR and Cone params
        self.declare_parameter("cone_size", 0.10)
        self.declare_parameter("cone_height", 0.30)

        # DBSCAN Parameters
        # Neighbourhood Scan Size
        # > 1m otherwise clusters smaller non-cones objects
        # Number of points required to form a neighbourhood
        self.declare_parameter("epsilon", 0.8)
        # 2-3 allows for long-range cone detection
        self.declare_parameter("min_points", 2)

        self.log_level = self.get_parameter("log_level").value
        log_level: int = getattr(rclpy.impl.logging_severity.LoggingSeverity, self.log_level.upper())
        self.get_logger().set_level(log_level)

        self.lidar_range = self.get_parameter("lidar_range").value
        self.min_range = self.get_parameter("min_range").value

        self.cone_size = self.get_parameter("cone_size").value
        self.cone_height = self.get_parameter("cone_height").value

        self.epsilon = self.get_parameter("epsilon").value
        self.min_points = self.get_parameter("min_points").value

    def callback(self, msg: PointCloud2):
        """
        Callback function for when point cloud data is received.
        The message is the object points extracted from the GroundPlaneSegmenter node (the green ones)

        Args:
            point_cloud_msg (PointCloud2): The point cloud message.
        """
        start_time: float = time.perf_counter()

        # Step 1

        # Convert PointCloud2 message from LiDAR sensor to numpy array
        dtype_list: list = fields_to_dtype(msg.fields, msg.point_step)  # x, y, z, intensity, ring
        object_points: np.ndarray = np.frombuffer(msg.data, dtype_list)

        # Step 4

        # Compute point normals
        point_norms = np.linalg.norm([object_points["x"], object_points["y"]], axis=0)

        # Step 5

        # Remove points that are outside of lidar range or min range
        mask = (point_norms < self.lidar_range) & (point_norms > self.min_range) # & (point_norms != 0)
        point_norms = point_norms[mask]
        object_points = object_points[mask]

        # Step 10: Cluster objects DBScan

        object_centers, objects = self.group_points(object_points)

        # for each object, colour (intensity) differently
        # for i, obj in enumerate(objects):
        #     obj["intensity"] = i*10 # 10x scalar to make changes between colour 1, 2, 3 etc more obvious

        # Step 12

        cone_clusters, cone_locations = self.cone_filter(objects, object_centers)

        # If no cones were detected, return
        if len(cone_locations) == 0:
            return

        # Convert cone locations to ConeDetection messages and publish
        detected_cones: list = [cone_msg(cone[0], cone[1]) for cone in cone_locations]
        detection_msg = ConeDetectionStamped(header=msg.header, cones=detected_cones)
        self.detection_publisher.publish(detection_msg)

        ## FOR VISUALS
        # create pointcloud of all clusters
        cone_clusters = np.concatenate(cone_clusters)
        new_point_cloud_msg = array_to_pointcloud2(msg, cone_clusters, dtype_list)
        self.point_cloud_publisher_debug.publish(new_point_cloud_msg)

        # Calculate runtime statistics
        self.fps.next_iter()
        self.get_logger().debug(
            f"Process Time: {round(time.perf_counter() - start_time, 4)}\t| EST. FPS: {round(self.fps.fps(), 2)}",
            throttle_duration_sec=0.5,
        )


    def group_points(self, object_points):
        # Cluster object points
        clustering = DBSCAN(eps=self.epsilon, min_samples=self.min_points).fit(
            np.column_stack((object_points["x"], object_points["y"]))
        )
        labels = clustering.labels_

        # All object ids
        unq_labels = np.unique(labels)[1:]  # Noise cluster -1 (np.unique sorts)

        objects = np.empty(unq_labels.size, dtype=object)
        object_centers = np.empty((unq_labels.size, 3))
        for idx, label in enumerate(unq_labels):
            objects[idx] = object_points[np.where(labels == label)]
            object_centers[idx] = np.mean(
                np.column_stack((objects[idx]["x"], objects[idx]["y"], objects[idx]["z"])), axis=0
            )

        return object_centers, objects


    def cone_filter(self, objects, object_centers):
        # size, filter out clusters with >15 points
        # cluster radius, filter out clusters >0.10m radius

        # iterate through clusters
            # get size of points in cluster
            # size is number of points in cluster
            # get radius of cluster
            # radius is max distance from center to any point

            # if size > 15 or diameter > 0.10
                # ignore

        # initialise a mask of all False for each object
        mask = np.zeros(len(objects), dtype=bool)
        # iterate through objects
        for i in range(len(objects)):
            if objects[i].size > 15:
                continue
            
            # object = [[x, y, z, intensity], [x, y, z, intensity], ...
            dists = np.linalg.norm(np.array([objects[i]["x"], objects[i]["y"]]).T - object_centers[i][:2], axis=1)
            # dists = [0.1, 0.2, 0.3, ...]

            if np.max(dists) > self.cone_size:
                continue

            # this object is a cone
            mask[i] = True

        # extract cone clusters where mask is True (meets size and diameter requirements)
        cone_clusters = objects[mask]
        cone_locations = object_centers[mask]

        return cone_clusters, cone_locations



def main(args=None):
    rclpy.init(args=args)
    node = LiDARDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
