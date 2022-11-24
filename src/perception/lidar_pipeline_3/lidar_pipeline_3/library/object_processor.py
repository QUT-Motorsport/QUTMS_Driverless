import numpy as np
from sklearn.cluster import DBSCAN

from . import point_cloud_processor as pcp
from .. import constants as const


def group_points(object_points):
    EPSILON = 0.6  # Neighbourhood Scan Size 0.1: +0Hz, 0.5: -2Hz, 1 -3Hz:
    MIN_POINTS = 3  # Number of points required to form a neighbourhood

    # Cluster object points
    clustering = DBSCAN(eps=EPSILON, min_samples=MIN_POINTS).fit(
        np.column_stack([object_points["x"], object_points["y"]])
    )
    labels = clustering.labels_

    # All object ids
    unq_labels = np.unique(labels)

    objects = np.empty(unq_labels.size, dtype=object)
    object_centers = np.empty((unq_labels.size, 3))
    for idx, label in enumerate(unq_labels):
        objects[idx] = object_points[np.where(labels == label)]
        object_centers[idx] = np.mean(
            np.column_stack([objects[idx]["x"], objects[idx]["y"], objects[idx]["z"]]), axis=0
        )

    return object_centers, objects


def reconstruct_objects_2(point_cloud, segments, bins, object_centers, objects):
    obj_norms = np.linalg.norm(object_centers, axis=1)
    obj_segs, obj_bins = pcp.get_discretised_positions(object_centers[:, 0], object_centers[:, 1], obj_norms)

    # Upside down floor devision
    bin_search_half = -((const.CONE_DIAM // const.BIN_SIZE) // -2)
    seg_widths = -((2 * (const.BIN_SIZE * obj_bins) * np.tan(const.DELTA_ALPHA / 2)) // -2)
    seg_search_half = np.floor_divide(const.CONE_DIAM, seg_widths)

    reconstructed_objs = []
    for i in range(object_centers.shape[0]):
        curr_seg = obj_segs[i]
        curr_bin = obj_bins[i]

        segments_ind = (curr_seg - seg_search_half[i] <= segments) * (segments <= curr_seg + seg_search_half[i])
        bins_ind = (curr_bin - bin_search_half <= bins) * (bins <= curr_bin + bin_search_half)
        search_points = point_cloud[segments_ind * bins_ind]

        distances = np.linalg.norm(
            np.column_stack((search_points["x"], search_points["y"])) - object_centers[i], axis=1
        )
        matching_points = search_points[distances <= const.CONE_DIAM / 2]
        matching_points = np.vstack((matching_points, objects[i]))

        zeros = np.zeros((objects[i].shape[0], 1))
        test = np.hstack([objects[i], zeros])
        obj = np.vstack([xyz[norms <= CONE_DIAM / 2], test])
        unq_obj = np.unique(obj, axis=0)

        reconstructed_objs.append(matching_points)  # add error margin?

    return reconstructed_objs


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
