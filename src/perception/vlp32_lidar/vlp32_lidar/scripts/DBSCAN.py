# Modules
import numpy as np
from sklearn.cluster import DBSCAN
from typing import List

# Density-Based Spatial Clustering of Applications with Noise (DBSCAN)
def get_objects(object_points: List[List[List]]) -> List[List]:
    EPSILON = 0.4  # Neighbourhood Scan Size
    MIN_POINTS = 3 # Number of points required to form a neighbourhood

    clustering = DBSCAN(eps = EPSILON, min_samples = MIN_POINTS).fit(np.array(object_points))
    cluster_count = max(clustering.labels_) + 1
    clusters: List[List] = [[] for i in range(cluster_count)]

    for i in range(len(clustering.labels_)):
        label: np.ndarray = clustering.labels_[i]
        point: List[List] = object_points[i]
        if label != -1:
            clusters[label].append(point)

    for i in range(len(clusters)):
        x_cluster: List = [coords[0] for coords in clusters[i]]
        y_cluster: List = [coords[1] for coords in clusters[i]]
        z_cluster: List = [coords[2] for coords in clusters[i]]
        x_mean: float = sum(x_cluster) / len(x_cluster)
        y_mean: float = sum(y_cluster) / len(y_cluster)
        z_mean: float = sum(z_cluster) / len(z_cluster)
        clusters[i] = [x_mean, y_mean, z_mean]

    return clusters

# Notes
# 1. This version of init_DBSCAN has disabled noise, and clusters get overwritten
#    by cluster_centers to increase the performance of the algorithm. (Refer to 
#    GIT history for deprecated code that offered noise and preserved clusters).
# 2. z_cluster and z_mean: I don't think we need to know the height value of the
#    center of a cluster. We are reconstructing an unbounded cylinder at each
#    cetner - not a sphere, so the height should not matter. That said, I believe
#    Monash University uses it - but they're a bunch of monkeys anyways. 
# 3. Neighbourhood scan size should be roughly the diameter of the cones that are
#    used on the track.
