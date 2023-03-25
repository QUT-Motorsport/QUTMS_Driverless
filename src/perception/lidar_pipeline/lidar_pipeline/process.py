import math

import numpy as np
from sklearn.cluster import DBSCAN

from sensor_msgs.msg import PointField

from .library.cy_library import total_least_squares as tls

# Algorithm Parameters | Don't forget algorithm tuning
LIDAR_RANGE = 25  # Max range of points to process (metres)
DELTA_ALPHA = (2 * math.pi) / 128  # Delta angle of segments
BIN_SIZE = 0.14  # Size of bins
T_M = 2 * math.pi / 148  # (2 * math.pi) / (152*2)           # Max angle that will be considered for ground lines
T_M_SMALL = 0  # Angle considered to be a small slope
T_B = 0.05  # Max y-intercept for a ground plane line
T_RMSE = 0.2  # Threshold of the Root Mean Square Error of the fit (Recommended: 0.2 - 0.5)
REGRESS_BETWEEN_BINS = True  # Determines if regression for ground lines should occur between two
# neighbouring bins when they're described by different lines
T_D_GROUND = 0.125  # 0.15 # Maximum distance between point and line to be considered part of ground plane # tune this
# changed from 0.1, ^^ also, the higher this value, the more low object points it will mark as ground BUT this makes dbscan faster
T_D_MAX = 100  # Maximum distance a point can be from the origin to even be considered as
# a ground point. Otherwise it's labelled as a non-ground point.
CPU_UTILISATION = 0.90  # Percentage of CPU Cores to use for multiprocessing ground plane mapping (0.0 - 1.0)
CONE_DIAM = 0.15
CONE_HEIGHT = 0.30

LIDAR_HEIGHT_ABOVE_GROUND = 0.15
LIDAR_VERTICAL_RES = 1.25 * (math.pi / 180)  # 1.25 degrees in between each point
LIDAR_HORIZONTAL_RES = 0.05 * (math.pi / 180)  # NEW
LHAG_ERR = 0.25

HACH_LOWER_ERR = 0.3  # 0.087 - 0.3 < 0 so min bound should probably just be zero lol
HACH_UPPER_ERR = CONE_HEIGHT  # - 0.025

# Derived Parameters
SEGMENT_COUNT = math.ceil(2 * math.pi / DELTA_ALPHA)
BIN_COUNT = math.ceil(LIDAR_RANGE / BIN_SIZE)
HALF_AREA_CONE_HEIGHT = CONE_HEIGHT * (2 - math.sqrt(2)) / 2  # 0.08787

# Expected number of points on a cone at a given distance
NUMER = CONE_HEIGHT * CONE_DIAM
DENOM = 8 * math.tan(LIDAR_VERTICAL_RES / 2) * math.tan(LIDAR_HORIZONTAL_RES / 2)

EPSILON = 0.6  # Neighbourhood Scan Size 0.1: +0Hz, 0.5: -2Hz, 1 -3Hz:
MIN_POINTS = 2  # Number of points required to form a neighbourhood


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


def get_discretised_positions(x, y, point_norms):
    # Calculating the segment index for each point
    segments_idx = np.arctan2(y, x) / DELTA_ALPHA
    np.nan_to_num(segments_idx, copy=False, nan=((np.pi / 2) / DELTA_ALPHA))  # Limit arctan x->inf = pi/2

    # Calculating the bin index for each point
    bins_idx = point_norms / BIN_SIZE

    # Stacking arrays segments_idx, bins_idx, point_norms, and xyz coords into one array
    return segments_idx.astype(int, copy=False), bins_idx.astype(int, copy=False)


# In LPP 2, np.absolute(z) is used. I'm not sure why this was the case.
# If a point is negative, i.e., low, that's still valid.
def get_prototype_points(z, segments, bins, point_norms):

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
def fit_error(m, b, points):
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
def get_bin(norm, BIN_SIZE):
    return math.floor(norm / BIN_SIZE)


# start and end points are used in visualisation
# The Incremental Algorithm
def get_ground_lines(proto_seg_points):
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

            m_b_check = abs(m_new) <= T_M and (abs(m_new) > T_M_SMALL or abs(b_new) <= T_B)
            if not (m_b_check and fit_error(m_new, b_new, new_line_points) <= T_RMSE):
                new_line_points.pop()  # Remove the point we just added

                [m_new, b_new] = tls.fit_line(new_line_points)

                m_b_check = abs(m_new) <= T_M and (abs(m_new) > T_M_SMALL or abs(b_new) <= T_B)
                if m_b_check and fit_error(m_new, b_new, new_line_points) <= T_RMSE:
                    estimated_lines.append(
                        (
                            m_new,
                            b_new,
                            new_line_points[0],
                            new_line_points[-1],
                            get_bin(new_line_points[0][0], BIN_SIZE),
                        )
                    )
                    lines_created += 1

                new_line_points = []

                if REGRESS_BETWEEN_BINS:
                    idx -= 2
                else:
                    idx -= 1

        else:
            if (
                len(new_line_points) == 0
                or math.atan((new_point[1] - new_line_points[-1][1]) / (new_point[0] - new_line_points[-1][0])) <= T_M
            ):
                new_line_points.append(new_point)

        idx += 1

    if len(new_line_points) > 1 and m_new != None and b_new != None:
        estimated_lines.append(
            (m_new, b_new, new_line_points[0], new_line_points[-1], get_bin(new_line_points[0][0], BIN_SIZE))
        )

    # If no ground lines were identified in segment, return 0
    if len(estimated_lines) > 0:
        return estimated_lines
    else:
        return 0


def get_ground_plane_single_core(proto_segs_arr, proto_segs):
    # Computing the ground plane
    ground_plane = np.zeros(SEGMENT_COUNT, dtype=object)  # should it be vector of dtype, or matrix of nums?

    for segment_counter in range(len(proto_segs_arr)):
        proto_seg_points = proto_segs_arr[segment_counter].tolist()
        ground_plane[proto_segs[segment_counter]] = get_ground_lines(proto_seg_points)

    return ground_plane


def map_segments(ground_plane):
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


def sort_segments(segments, seg_bin_z_ind):
    segments_sorted = segments[seg_bin_z_ind]

    # Indicies where segments differ
    seg_sorted_diff = np.where(segments_sorted[:-1] != segments_sorted[1:])[0] + 1

    # Indicies where segments differ (appending first element at 0)
    seg_sorted_ind = np.empty(seg_sorted_diff.size + 1, dtype=int)
    seg_sorted_ind[0] = 0
    seg_sorted_ind[1:] = seg_sorted_diff

    return seg_sorted_ind, segments_sorted


def label_points(point_heights, segments, bins, seg_bin_z_ind, ground_plane):
    ground_plane = map_segments(ground_plane)

    # Get indices where sorted segments differ
    seg_sorted_ind, segments_sorted = sort_segments(segments, seg_bin_z_ind)

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

    discretised_ground_heights = ((BIN_SIZE * bins) * ground_lines_arr[:, 0]) + ground_lines_arr[:, 1]
    point_line_dists = (
        point_heights - discretised_ground_heights
    )  # should there be an abs() here? no, read comment below

    point_labels = point_line_dists > T_D_GROUND  # if close enough, or simply lower than line
    return point_labels, ground_lines_arr


def group_points(object_points):
    # Cluster object points
    clustering = DBSCAN(eps=EPSILON, min_samples=MIN_POINTS).fit(
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


def reconstruct_objects(ground_points, ground_segments, ground_bins, object_centers, objects):
    obj_norms = np.linalg.norm(object_centers[:, :2], axis=1)
    obj_segs, obj_bins = get_discretised_positions(object_centers[:, 0], object_centers[:, 1], obj_norms)

    # Upside down floor devision
    bin_search_half = -((CONE_DIAM // BIN_SIZE) // -2)
    seg_widths = -((2 * (BIN_SIZE * obj_bins) * np.tan(DELTA_ALPHA / 2)) // -2)
    seg_search_half = np.floor_divide(CONE_DIAM, seg_widths)

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
            in_range_points = search_points[distances <= CONE_DIAM / 2]
            matching_points = np.append(matching_points, in_range_points)

        reconstructed_centers[i] = np.mean(
            np.column_stack((matching_points["x"], matching_points["y"], matching_points["z"])), axis=0
        )
        reconstructed_objs[i] = matching_points  # add error margin?

    return obj_segs, obj_bins, reconstructed_objs, reconstructed_centers, avg_object_intensity


# Number of points expected to be on a cone at a given distance
def get_expected_point_count(distance):
    return NUMER / (np.square(distance) * DENOM)


def cone_filter(
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
    discretised_ground_heights = ((BIN_SIZE * bins[seg_bin_ind]) * ground_lines_arr[seg_bin_ind, 0]) + ground_lines_arr[
        seg_bin_ind, 1
    ]
    object_line_dists = np.abs(object_centers[:, 2] - discretised_ground_heights)

    # Upper bound cone height, lower bound take err margin
    f1_matching_ind = (HALF_AREA_CONE_HEIGHT - HACH_LOWER_ERR <= object_line_dists) * (
        object_line_dists <= HALF_AREA_CONE_HEIGHT + HACH_UPPER_ERR
    )

    filtered_rec_centers = reconstructed_centers[f1_matching_ind]
    filtered_rec_objects = reconstructed_objects[f1_matching_ind]
    filtered_avg_intensity = avg_object_intensity[f1_matching_ind]

    # Filter 2: How many points do we expect to be on a cone at a given distance?
    rec_norms = np.linalg.norm(filtered_rec_centers[:, :2], axis=1)
    rec_point_counts = np.array([len(rec) for rec in filtered_rec_objects])
    expected_point_counts = get_expected_point_count(rec_norms)

    f2_matching_ind = (0.3 * expected_point_counts <= rec_point_counts) * (
        rec_point_counts <= 1.5 * expected_point_counts
    )

    return (
        filtered_rec_centers[f2_matching_ind],
        filtered_rec_objects[f2_matching_ind],
        filtered_avg_intensity[f2_matching_ind],
    )
