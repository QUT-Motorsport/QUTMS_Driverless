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
