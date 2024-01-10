import math
import time

import numpy as np
from sklearn.cluster import DBSCAN

import rclpy
from rclpy.node import Node

from driverless_msgs.msg import Cone, ConeDetectionStamped
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField

from driverless_common.common import QOS_ALL, FPSHandler

from .library.cy_library import total_least_squares as tls


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
        self.create_subscription(PointCloud2, "/velodyne_points", self.callback, QOS_ALL)
        self.detection_publisher = self.create_publisher(ConeDetectionStamped, "/lidar/cone_detection", 1)

        # Log info
        self.get_logger().info("---LiDAR detector node initialised---")

    def initialise_params(self):
        # declare parameters
        self.declare_parameter("log_level", "DEBUG")
        self.declare_parameter("lidar_range", 25)
        self.declare_parameter("delta_alpha_ang", 128)
        self.declare_parameter("bin_size", 0.14)
        self.declare_parameter("t_m_ang", 148)
        self.declare_parameter("t_m_small", 0)
        self.declare_parameter("t_b", 0.05)
        self.declare_parameter("t_rmse", 0.2)
        self.declare_parameter("regress_between_bins", True)
        self.declare_parameter("t_d_ground", 0.125)
        self.declare_parameter("t_d_max", 100)
        self.declare_parameter("cpu_utilisation", 0.90)
        self.declare_parameter("cone_diam", 0.15)
        self.declare_parameter("cone_height", 0.30)
        self.declare_parameter("lidar_height_above_ground", 0.15)
        self.declare_parameter("lidar_vertical_res_val", 1.25)
        self.declare_parameter("lidar_horizontal_res_val", 0.05)
        self.declare_parameter("lhag_err", 0.25)
        self.declare_parameter("hach_lower_err", 0.35)
        self.declare_parameter("hach_upper_err", 0.15)
        self.declare_parameter("epsilon", 1.2)
        self.declare_parameter("min_points", 2)

        self.log_level = self.get_parameter("log_level").value
        log_level: int = getattr(rclpy.impl.logging_severity.LoggingSeverity, self.log_level.upper())
        self.get_logger().set_level(log_level)

        self.lidar_range = self.get_parameter("lidar_range").value
        self.delta_alpha = (2 * math.pi) / self.get_parameter("delta_alpha_ang").value
        self.bin_size = self.get_parameter("bin_size").value
        self.t_m = (2 * math.pi) / self.get_parameter("t_m_ang").value
        self.t_m_small = self.get_parameter("t_m_small").value
        self.t_b = self.get_parameter("t_b").value
        self.t_rmse = self.get_parameter("t_rmse").value
        self.regress_between_bins = self.get_parameter("regress_between_bins").value
        self.t_d_ground = self.get_parameter("t_d_ground").value
        self.t_d_max = self.get_parameter("t_d_max").value
        self.cpu_utilisation = self.get_parameter("cpu_utilisation").value
        self.cone_diam = self.get_parameter("cone_diam").value
        self.cone_height = self.get_parameter("cone_height").value
        self.lidar_height_above_ground = self.get_parameter("lidar_height_above_ground").value
        self.lidar_vertical_res = self.get_parameter("lidar_vertical_res_val").value * (math.pi / 180)
        self.lidar_horizontal_res = self.get_parameter("lidar_horizontal_res_val").value * (math.pi / 180)
        self.lhag_err = self.get_parameter("lhag_err").value
        self.hach_lower_err = self.get_parameter("hach_lower_err").value
        self.hach_upper_err = self.get_parameter("hach_upper_err").value
        self.epsilon = self.get_parameter("epsilon").value
        self.min_points = self.get_parameter("min_points").value

        # Derived Parameters
        self.segment_count = math.ceil(2 * math.pi / self.delta_alpha)
        self.bin_count = math.ceil(self.lidar_range / self.bin_size)
        self.half_area_cone_height = self.cone_height * (2 - math.sqrt(2)) / 2

        self.numer = self.cone_height * self.cone_diam
        self.denom = 8 * math.tan(self.lidar_vertical_res / 2) * math.tan(self.lidar_horizontal_res / 2)

        self.get_logger().info(
            "PARAMS: Lidar Range: "
            + str(self.lidar_range)
            + "m"
            + "\t| Delta Alpha: "
            + str(self.delta_alpha)
            + "rad"
            + "\t| Bin Size: "
            + str(self.bin_size)
            + "m"
            + "\t| T_m: "
            + str(self.t_m)
            + "rad"
            + "\t| T_b: "
            + str(self.t_b)
            + "m"
            + "\t| T_rmse: "
            + str(self.t_rmse)
            + "m"
            + "\t| Regress Between Bins: "
            + str(self.regress_between_bins)
            + "\t| T_d_ground: "
            + str(self.t_d_ground)
            + "m"
            + "\t| T_d_max: "
            + str(self.t_d_max)
            + "m"
            + "\t| CPU Utilisation: "
            + str(self.cpu_utilisation)
            + "\t| Cone Diameter: "
            + str(self.cone_diam)
            + "m"
            + "\t| Cone Height: "
            + str(self.cone_height)
            + "m"
            + "\t| Lidar Height Above Ground: "
            + str(self.lidar_height_above_ground)
            + "m"
            + "\t| Lidar Vertical Resolution: "
            + str(self.lidar_vertical_res)
            + "rad"
            + "\t| Lidar Horizontal Resolution: "
            + str(self.lidar_horizontal_res)
            + "rad"
            + "\t| LHAG Error: "
            + str(self.lhag_err)
            + "m"
            + "\t| HACH Lower Error: "
            + str(self.hach_lower_err)
            + "m"
            + "\t| HACH Upper Error: "
            + str(self.hach_upper_err)
            + "m"
            + "\t| Epsilon: "
            + str(self.epsilon)
            + "\t| Min Points: "
            + str(self.min_points)
        )

    def callback(self, msg: PointCloud2):
        """
        Callback function for when point cloud data is received.

        Args:
            point_cloud_msg (PointCloud2): The point cloud message.
        """
        start_time: float = time.perf_counter()

        # Convert PointCloud2 message from LiDAR sensor to numpy array
        dtype_list: list = fields_to_dtype(msg.fields, msg.point_step)  # x, y, z, intensity, ring
        point_cloud: np.ndarray = np.frombuffer(msg.data, dtype_list)

        # Remove points behind car
        point_cloud = point_cloud[point_cloud["x"] > 0]

        # Remove points that are above the height of cones
        point_cloud = point_cloud[
            point_cloud["z"] < self.lhag_err * (self.lidar_height_above_ground + self.cone_height)
        ]

        # Compute point normals
        point_norms = np.linalg.norm([point_cloud["x"], point_cloud["y"]], axis=0)

        # Remove points that are outside of range or have a norm of 0
        mask = point_norms <= self.lidar_range  # & (point_norms != 0)
        point_norms = point_norms[mask]
        point_cloud = point_cloud[mask]

        # Get segments and prototype points
        segments, bins = self.get_discretised_positions(point_cloud["x"], point_cloud["y"], point_norms)
        proto_segs_arr, proto_segs, seg_bin_z_ind = self.get_prototype_points(
            point_cloud["z"], segments, bins, point_norms
        )

        # Extract ground plane
        ground_plane = self.get_ground_plane_single_core(proto_segs_arr, proto_segs)

        # Label points
        point_labels, ground_lines_arr = self.label_points(
            point_cloud["z"], segments, bins, seg_bin_z_ind, ground_plane
        )
        object_points = point_cloud[point_labels]

        if object_points.size == 0:
            self.get_logger().info("No objects points detected")
            return []

        object_centers, objects = self.group_points(object_points)

        ground_points = point_cloud[~point_labels]
        (
            obj_segs,
            obj_bins,
            reconstructed_objects,
            reconstructed_centers,
            avg_object_intensity,
        ) = self.reconstruct_objects(
            ground_points, segments[~point_labels], bins[~point_labels], object_centers, objects
        )

        cone_locations, cone_points, cone_intensities = self.cone_filter(
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
        self.fps.next_iter()
        self.get_logger().debug(
            f"Process Time: {round(time.perf_counter() - start_time, 4)}\t| EST. FPS: {round(self.fps.fps(), 2)}",
            throttle_duration_sec=0.5,
        )

        # If no cones were detected, return
        if len(cone_locations) == 0:
            return

        # Convert cone locations to ConeDetection messages and publish
        detected_cones: list = [cone_msg(cone[0], cone[1]) for cone in cone_locations]
        detection_msg = ConeDetectionStamped(header=msg.header, cones=detected_cones)
        self.detection_publisher.publish(detection_msg)

    def get_discretised_positions(self, x, y, point_norms):
        # Calculating the segment index for each point
        segments_idx = np.arctan2(y, x) / self.delta_alpha
        np.nan_to_num(segments_idx, copy=False, nan=((np.pi / 2) / self.delta_alpha))  # Limit arctan x->inf = pi/2

        # Calculating the bin index for each point
        bins_idx = point_norms / self.bin_size

        # Stacking arrays segments_idx, bins_idx, point_norms, and xyz coords into one array
        return segments_idx.astype(int, copy=False), bins_idx.astype(int, copy=False)

    # In LPP 2, np.absolute(z) is used. I'm not sure why this was the case.
    # If a point is negative, i.e., low, that's still valid.
    def get_prototype_points(self, z, segments, bins, point_norms):

        # Indicies sorted by segments, then bins, then z (height)
        seg_bin_z_ind = np.lexsort((z, bins, segments))

        # Indicies where neighbouring bins in array are different
        bin_diff_ind = np.where((bins[seg_bin_z_ind])[:-1] != (bins[seg_bin_z_ind])[1:])[0] + 1

        # Indicies of prototype points
        proto_sorted_ind = np.empty(bin_diff_ind.size + 1, dtype=int)
        proto_sorted_ind[0] = seg_bin_z_ind[0]
        proto_sorted_ind[1:] = seg_bin_z_ind[bin_diff_ind]

        # Prototype points and segment idx corresponding to each
        proto_points = np.column_stack((point_norms[proto_sorted_ind], z[proto_sorted_ind]))
        proto_segments = segments[proto_sorted_ind]

        # Indicies where neighbouring prototype_segments value in array are different
        proto_segs_diff = np.where(proto_segments[:-1] != proto_segments[1:])[0] + 1

        # Prototype points split into subarrays for each segment
        proto_segs_arr = np.split(proto_points, proto_segs_diff)

        return proto_segs_arr, proto_segments[np.concatenate((np.array([0]), proto_segs_diff))], seg_bin_z_ind

    # Returns the RMSE of a line fit to a set of points
    def fit_error(self, m, b, points):
        num_points = len(points)

        sse = 0
        for i in range(num_points):
            x = points[i][0]
            best_fit = m * x + b

            observed = points[i][1]
            sse += (best_fit - observed) ** 2

        # root mean square return
        return math.sqrt(sse / num_points)

    # Returns bin idx of a point from its norm
    def get_bin(self, norm, BIN_SIZE):
        return math.floor(norm / BIN_SIZE)

    # start and end points are used in visualisation
    # The Incremental Algorithm
    def get_ground_lines(self, proto_seg_points):
        estimated_lines = []
        new_line_points = []
        lines_created = 0

        idx = 0
        while idx < len(proto_seg_points):
            m_new = None
            b_new = None

            new_point = proto_seg_points[idx]
            if len(new_line_points) >= 2:
                new_line_points.append(new_point)

                [m_new, b_new] = tls.fit_line(new_line_points)

                m_b_check = abs(m_new) <= self.t_m and (abs(m_new) > self.t_m_small or abs(b_new) <= self.t_b)
                if not (m_b_check and self.fit_error(m_new, b_new, new_line_points) <= self.t_rmse):
                    new_line_points.pop()  # Remove the point we just added

                    [m_new, b_new] = tls.fit_line(new_line_points)

                    m_b_check = abs(m_new) <= self.t_m and (abs(m_new) > self.t_m_small or abs(b_new) <= self.t_b)
                    if m_b_check and self.fit_error(m_new, b_new, new_line_points) <= self.t_rmse:
                        estimated_lines.append(
                            (
                                m_new,
                                b_new,
                                new_line_points[0],
                                new_line_points[-1],
                                self.get_bin(new_line_points[0][0], self.bin_size),
                            )
                        )
                        lines_created += 1

                    new_line_points = []

                    if self.regress_between_bins:
                        idx -= 2
                    else:
                        idx -= 1

            else:
                if (
                    len(new_line_points) == 0
                    or math.atan((new_point[1] - new_line_points[-1][1]) / (new_point[0] - new_line_points[-1][0]))
                    <= self.t_m
                ):
                    new_line_points.append(new_point)

            idx += 1

        if len(new_line_points) > 1 and m_new != None and b_new != None:
            estimated_lines.append(
                (
                    m_new,
                    b_new,
                    new_line_points[0],
                    new_line_points[-1],
                    self.get_bin(new_line_points[0][0], self.bin_size),
                )
            )

        # If no ground lines were identified in segment, return 0
        if len(estimated_lines) > 0:
            return estimated_lines
        else:
            return 0

    def get_ground_plane_single_core(self, proto_segs_arr, proto_segs):
        # Computing the ground plane
        ground_plane = np.zeros(self.segment_count, dtype=object)  # should it be vector of dtype, or matrix of nums?

        for segment_counter in range(len(proto_segs_arr)):
            proto_seg_points = proto_segs_arr[segment_counter].tolist()
            ground_plane[proto_segs[segment_counter]] = self.get_ground_lines(proto_seg_points)

        return ground_plane

    def map_segments(self, ground_plane):
        non_zeros = np.flatnonzero(ground_plane)

        mask = np.ones(ground_plane.size, dtype=bool)
        mask[non_zeros] = False
        zeros = np.arange(ground_plane.size)[mask]

        for idx in zeros:
            dists = np.abs(idx - non_zeros)
            wrap_dists = np.abs(ground_plane.size - dists)

            min_dist = np.min(dists)
            min_wrap_dist = np.min(wrap_dists)
            if min_dist <= min_wrap_dist:
                closest_idx = non_zeros[np.min(np.where(dists == min_dist))]
            else:
                closest_idx = non_zeros[np.min(np.where(wrap_dists == min_wrap_dist))]

            ground_plane[idx] = ground_plane[closest_idx]

        return ground_plane

    def sort_segments(self, segments, seg_bin_z_ind):
        segments_sorted = segments[seg_bin_z_ind]

        # Indicies where segments differ
        seg_sorted_diff = np.where(segments_sorted[:-1] != segments_sorted[1:])[0] + 1

        # Indicies where segments differ (appending first element at 0)
        seg_sorted_ind = np.empty(seg_sorted_diff.size + 1, dtype=int)
        seg_sorted_ind[0] = 0
        seg_sorted_ind[1:] = seg_sorted_diff

        return seg_sorted_ind, segments_sorted

    def label_points(self, point_heights, segments, bins, seg_bin_z_ind, ground_plane):
        ground_plane = self.map_segments(ground_plane)

        # Get indices where sorted segments differ
        seg_sorted_ind, segments_sorted = self.sort_segments(segments, seg_bin_z_ind)

        ground_lines_arr = np.empty((point_heights.shape[0], 2))
        for segment_idx in segments_sorted[seg_sorted_ind]:
            ground_set = ground_plane[segment_idx]
            seg_eq_idx = segments == segment_idx
            ground_lines_arr[seg_eq_idx.nonzero()[0], :] = np.array([ground_set[0][0], ground_set[0][1]])
            # For each line in segment
            for ground_line in ground_set:
                curr_bin = ground_line[4]
                line_ind = (seg_eq_idx & (bins >= curr_bin)).nonzero()[0]
                ground_lines_arr[line_ind, :] = np.array([ground_line[0], ground_line[1]])

        discretised_ground_heights = ((self.bin_size * bins) * ground_lines_arr[:, 0]) + ground_lines_arr[:, 1]
        point_line_dists = (
            point_heights - discretised_ground_heights
        )  # should there be an abs() here? no, read comment below

        point_labels = point_line_dists > self.t_d_ground  # if close enough, or simply lower than line
        return point_labels, ground_lines_arr

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

    def reconstruct_objects(self, ground_points, ground_segments, ground_bins, object_centers, objects):
        obj_norms = np.linalg.norm(object_centers[:, :2], axis=1)
        obj_segs, obj_bins = self.get_discretised_positions(object_centers[:, 0], object_centers[:, 1], obj_norms)

        # Upside down floor devision
        bin_search_half = -((self.cone_diam // self.bin_size) // -2)
        seg_widths = -((2 * (self.bin_size * obj_bins) * np.tan(self.delta_alpha / 2)) // -2)
        seg_search_half = np.floor_divide(self.cone_diam, seg_widths)

        # do i even car about all the points in a recon object? wouldnt i just want the cetner, and num points?
        reconstructed_objs = np.empty(object_centers.shape[0], dtype=object)
        reconstructed_centers = np.empty((object_centers.shape[0], 3))
        avg_object_intensity = np.empty(object_centers.shape[0])
        for i in range(object_centers.shape[0]):
            matching_points = objects[i]
            avg_object_intensity[i] = np.mean(matching_points["intensity"])

            curr_seg = obj_segs[i]
            curr_bin = obj_bins[i]

            segments_ind = (curr_seg - seg_search_half[i] <= ground_segments) * (
                ground_segments <= curr_seg + seg_search_half[i]
            )
            bins_ind = (curr_bin - bin_search_half <= ground_bins) * (ground_bins <= curr_bin + bin_search_half)
            search_points = ground_points[segments_ind * bins_ind]

            if search_points.size > 0:
                distances = np.linalg.norm(
                    np.column_stack((search_points["x"], search_points["y"])) - object_centers[i, :2], axis=1
                )
                in_range_points = search_points[distances <= self.cone_diam / 2]
                matching_points = np.append(matching_points, in_range_points)

            reconstructed_centers[i] = np.mean(
                np.column_stack((matching_points["x"], matching_points["y"], matching_points["z"])), axis=0
            )
            reconstructed_objs[i] = matching_points  # add error margin?

        return obj_segs, obj_bins, reconstructed_objs, reconstructed_centers, avg_object_intensity

    # Number of points expected to be on a cone at a given distance
    def get_expected_point_count(self, distance):
        return self.numer / (np.square(distance) * self.denom)

    def cone_filter(
        self,
        segments,
        bins,
        ground_lines_arr,
        obj_segs,
        obj_bins,
        object_centers,
        reconstructed_objects,
        reconstructed_centers,
        avg_object_intensity,
    ):

        # Filter 1: Height of object compared to expected height of cone
        seg_bin_ind = (obj_segs == segments) * (obj_bins == bins)

        # i think i chose to use object center here instead of rec cause i thought that implied
        # a line was guranteed to have been computed, a thus exist in ground_lines_arr
        # but huge angled walls can cause an object center to not actually be on any of its points
        # so maybe use reconstructed instead? maybe have a try-catch to and ignore any objects that
        # don't have a line computed in their bin. they're probably not cones anyway
        discretised_ground_heights = (
            (self.bin_size * bins[seg_bin_ind]) * ground_lines_arr[seg_bin_ind, 0]
        ) + ground_lines_arr[seg_bin_ind, 1]
        object_line_dists = np.abs(object_centers[:, 2] - discretised_ground_heights)

        # Upper bound cone height, lower bound take err margin
        f1_matching_ind = (self.half_area_cone_height - self.hach_lower_err <= object_line_dists) * (
            object_line_dists <= self.half_area_cone_height + self.hach_upper_err
        )

        try:
            filtered_rec_centers = reconstructed_centers[f1_matching_ind]
        except IndexError:
            return np.empty((0, 3)), np.empty(0, dtype=object), np.empty(0)

        try:
            filtered_rec_objects = reconstructed_objects[f1_matching_ind]
        except IndexError:
            return np.empty((0, 3)), np.empty(0, dtype=object), np.empty(0)

        try:
            filtered_avg_intensity = avg_object_intensity[f1_matching_ind]
        except IndexError:
            return np.empty((0, 3)), np.empty(0, dtype=object), np.empty(0)

        # Filter 2: How many points do we expect to be on a cone at a given distance?
        rec_norms = np.linalg.norm(filtered_rec_centers[:, :2], axis=1)
        rec_point_counts = np.array([len(rec) for rec in filtered_rec_objects])
        expected_point_counts = self.get_expected_point_count(rec_norms)

        f2_matching_ind = (0.3 * expected_point_counts <= rec_point_counts) * (
            rec_point_counts <= 1.5 * expected_point_counts
        )

        return (
            filtered_rec_centers[f2_matching_ind],
            filtered_rec_objects[f2_matching_ind],
            filtered_avg_intensity[f2_matching_ind],
        )


def main(args=None):
    rclpy.init(args=args)
    node = LiDARDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
