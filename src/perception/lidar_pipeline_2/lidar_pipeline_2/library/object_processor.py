import point_cloud_processor as pcp

import numpy as np
from sklearn.cluster import DBSCAN

def group_points(object_points):
    EPSILON = 0.4  # Neighbourhood Scan Size
    MIN_POINTS = 3 # Number of points required to form a neighbourhood

    # Cluster object points
    clustering = DBSCAN(eps=EPSILON, min_samples=MIN_POINTS).fit(object_points)
    labels = clustering.labels_
    
    # All object ids
    unq_labels = np.unique(labels)
    
    objects = np.empty(unq_labels.size, dtype=object)
    object_centers = np.empty((unq_labels.size, 2))
    for idx, label in enumerate(unq_labels):
        objects[idx] = object_points[np.where(labels == label)]
        object_centers[idx] = np.mean(objects[idx], axis=0)
    
    return object_centers


def reconstruct_objects(object_centers, DELTA_ALPHA, CONE_DIAM, BIN_SIZE):
    center_norms = np.linalg.norm(object_centers, axis=1)
    segment_widths = 2 * np.multiply(center_norms, np.tan(DELTA_ALPHA / 2))
    num_segs_to_search = np.empty(segment_widths.size, dtype=int)
    np.ceil(np.divide(CONE_DIAM, segment_widths), out=num_segs_to_search, casting='unsafe')
    
    num_bins_to_search = CONE_DIAM / BIN_SIZE

    # Which segments / bins to search around
    center_segs, center_bins = pcp.get_discretised_positions_2(object_centers[:, 0], object_centers[:, 1], center_norms, DELTA_ALPHA, BIN_SIZE)
    
    
    print(object_centers)
    print(object_centers.shape)
    print(center_norms)
    print(center_norms.shape)
    print(segment_widths)
    print(segment_widths.shape)
    print(num_segs_to_search)
    print(type(num_segs_to_search))
    
    return None


def cone_filter(objects):
    pass

# If noise exists, it'll be the first object
# Do i need objects?
# can i simply just average without creating var for them
# and I need to filter noise out of centers