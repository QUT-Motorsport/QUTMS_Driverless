import numpy as np
from sklearn.cluster import DBSCAN

from . import point_cloud_processor as pcp
from .. import constants as const


def group_points(object_points):
    EPSILON = 0.6  # Neighbourhood Scan Size 0.1: +0Hz, 0.5: -2Hz, 1 -3Hz:
    MIN_POINTS = 2  # Number of points required to form a neighbourhood
    # move these into constants file

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


def reconstruct_objects_2(ground_points, ground_segments, ground_bins, object_centers, objects):
    obj_norms = np.linalg.norm(object_centers[:, :2], axis=1)
    obj_segs, obj_bins = pcp.get_discretised_positions(object_centers[:, 0], object_centers[:, 1], obj_norms)

    # Upside down floor devision
    bin_search_half = -((const.CONE_DIAM // const.BIN_SIZE) // -2)
    seg_widths = -((2 * (const.BIN_SIZE * obj_bins) * np.tan(const.DELTA_ALPHA / 2)) // -2)
    seg_search_half = np.floor_divide(const.CONE_DIAM, seg_widths)

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
            in_range_points = search_points[distances <= const.CONE_DIAM / 2]
            matching_points = np.append(matching_points, in_range_points)

        reconstructed_centers[i] = np.mean(
            np.column_stack((matching_points["x"], matching_points["y"], matching_points["z"])), axis=0
        )
        reconstructed_objs[i] = matching_points  # add error margin?

    return obj_segs, obj_bins, reconstructed_objs, reconstructed_centers, avg_object_intensity


def reconstruct_objects(point_cloud, object_centers, objects, DELTA_ALPHA, CONE_DIAM, BIN_SIZE):
    # center_norms = np.linalg.norm(object_centers, axis=1)
    # segment_widths = 2 * np.multiply(center_norms, np.tan(DELTA_ALPHA / 2))
    # num_segs_to_search = np.empty(segment_widths.size, dtype=int)
    # np.ceil(np.divide(CONE_DIAM, segment_widths), out=num_segs_to_search, casting="unsafe")

    # num_bins_to_search = CONE_DIAM / BIN_SIZE

    # Which segments / bins to search around
    # center_segs, center_bins = pcp.get_discretised_positions(object_centers[:, 0], object_centers[:, 1], center_norms)

    # Hacky shit, getting points in search area
    reconstructed_objects = []

    xyz = np.column_stack((point_cloud["x"], point_cloud["y"], point_cloud["z"]))
    for i in range(len(object_centers)):
        diff = xyz - np.append(object_centers[i], 0)
        norms = np.linalg.norm(diff, axis=1)

        zeros = np.zeros((objects[i].shape[0], 1))
        test = np.hstack([objects[i], zeros])
        obj = np.vstack([xyz[norms <= CONE_DIAM / 2], test])
        unq_obj = np.unique(obj, axis=0)
        # print(obj.size, unq_obj.size)
        reconstructed_objects.append(unq_obj)

    return reconstructed_objects


# Number of points expected to be on a cone at a given distance
def get_expected_point_count(distance):
    return const.NUMER / (np.square(distance) * const.DENOM)


# get height of object
# get ground line
# check height above ground line
# Since every single object will exists where points did,
# we have already know the ground line for the segment and bin
# of the object center. I doubt it's possible for an object center
# to be in a segment or bin where no points were due to the small
# epsilon in DBSCAN. If you introduce the noise cluster back, you will though
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
    discretised_ground_heights = (
        (const.BIN_SIZE * bins[seg_bin_ind]) * ground_lines_arr[seg_bin_ind, 0]
    ) + ground_lines_arr[seg_bin_ind, 1]
    object_line_dists = np.abs(object_centers[:, 2] - discretised_ground_heights)

    # Upper bound cone height, lower bound take err margin
    f1_matching_ind = (const.HALF_AREA_CONE_HEIGHT - const.HACH_LOWER_ERR <= object_line_dists) * (
        object_line_dists <= const.HALF_AREA_CONE_HEIGHT + const.HACH_UPPER_ERR
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


# if average is straight up higher than a cone, ignore it

# if distance between height of rec and ground is more than what you would expect the average height of points on a cone to be, filter out
# do like segments * bins and match for object centers

# If you do some integration on a triangle, you can find the height at which
# you expect 50% of the area to be, i.e., since more lidar points will hit the bottom of
# a triangle compared to the top, the average height of points will be lower than half
# the height of the triangle
# This is formula for the height of a triangle for 50% of its area
# h2 = h1 * (2 - sqtr(2)) / 2
# where h1 is the height of the triangle
