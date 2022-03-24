from sklearn.cluster import DBSCAN
import numpy as np

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

# If noise exists, it'll be the first object
# Do i need objects?
# can i simply just average without creating var for them
# and I need to filter noise out of centers