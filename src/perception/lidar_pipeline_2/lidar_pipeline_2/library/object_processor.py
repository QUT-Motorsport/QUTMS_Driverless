import numpy as np
from sklearn.cluster import DBSCAN

from . import point_cloud_processor as pcp


def group_points(object_points, EPS, MIN_SAMPLES):
    # Cluster object points
    clustering = DBSCAN(eps=EPS, min_samples=MIN_SAMPLES).fit(object_points)
    labels = clustering.labels_

    # All object ids
    unq_labels = np.unique(labels)

    objects = np.empty(unq_labels.size, dtype=object)
    object_centers = np.empty((unq_labels.size, 2))
    for idx, label in enumerate(unq_labels):
        objects[idx] = object_points[np.where(labels == label)]
        object_centers[idx] = np.mean(objects[idx], axis=0)

    return object_centers, objects


def reconstruct_objects(point_cloud, object_centers, objects, DELTA_ALPHA, CONE_DIAM, BIN_SIZE):
    center_norms = np.linalg.norm(object_centers, axis=1)
    segment_widths = 2 * np.multiply(center_norms, np.tan(DELTA_ALPHA / 2))
    num_segs_to_search = np.empty(segment_widths.size, dtype=int)
    np.ceil(np.divide(CONE_DIAM, segment_widths), out=num_segs_to_search, casting="unsafe")

    num_bins_to_search = CONE_DIAM / BIN_SIZE

    # Which segments / bins to search around
    # center_segs, center_bins = pcp.get_discretised_positions_2(
    #     object_centers[:, 0], object_centers[:, 1], center_norms, DELTA_ALPHA, BIN_SIZE
    # )

    # Hacky shit, getting points in search area
    reconstructed_objects = []

    for i in range(len(object_centers)):
        diff = point_cloud - np.append(object_centers[i], 0)
        norms = np.linalg.norm(diff, axis=1)

        zeros = np.zeros((objects[i].shape[0], 1))
        test = np.hstack([objects[i], zeros])
        obj = np.vstack([point_cloud[norms <= CONE_DIAM / 2], test])
        unq_obj = np.unique(obj, axis=0)
        # print(obj.size, unq_obj.size)
        reconstructed_objects.append(unq_obj)

    return reconstructed_objects


# If noise exists, it'll be the first object
# Do i need objects?
# can i simply just average without creating var for them
# and I need to filter noise out of centers
