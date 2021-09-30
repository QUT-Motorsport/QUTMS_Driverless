# Density-Based Spatial Clustering of Applications with Noise (DBSCAN)
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN

def init_DBSCAN(object_points):
    #print("--------------------")
    #print(object_points)
    epsilon = 0.275  # Neighbourhood scan size
    min_points = 2 # Minimum number of points required to form a neighbourhood
    object_points_np = np.array(object_points)
    clustering = DBSCAN(eps = epsilon, min_samples = min_points).fit(object_points_np)
    clusters = []
    cluster_count = max(clustering.labels_) + 1
    for i in range (cluster_count):
        clusters.append([])
    noise = []
    for i in range(len(clustering.labels_)):
        label = clustering.labels_[i]
        point = object_points[i]
        if label != -1:
            clusters[label].append(point)
        else:
            noise.append(point)

    cluster_centers = []
    # I don't acutually think we care about the height of the center
    # of the cluster. We just want to reconstruct a cyliner, not a sphere. 
    for i in range(len(clusters)):
        x_cluster = [coords[0] for coords in clusters[i]]
        y_cluster = [coords[1] for coords in clusters[i]]
        #z_cluster = [coords[2] for coords in clusters[i]]
        x_mean = sum(x_cluster) / len(x_cluster)
        y_mean = sum(y_cluster) / len(y_cluster)
        #z_mean = sum(z_cluster) / len(z_cluster)
        cluster_centers.append([x_mean, y_mean])

    return clusters, noise, cluster_centers

# Noise disabled
# Clusters gets overwritten by cluster_centers
def init_DBSCAN_2(object_points):
    EPSILON = 0.275  # Neighbourhood scan size
    MIN_POINTS = 2 # Minimum number of points required to form a neighbourhood
    clustering = DBSCAN(eps = EPSILON, min_samples = MIN_POINTS).fit(np.array(object_points))
    cluster_count = max(clustering.labels_) + 1
    clusters = [[] for i in range(cluster_count)]
    for i in range(len(clustering.labels_)):
        label = clustering.labels_[i]
        point = object_points[i]
        if label != -1:
            clusters[label].append(point)
            
    # I don't acutually think we care about the height of the center
    # of the cluster. We just want to reconstruct a cyliner, not a sphere. 
    for i in range(len(clusters)):
        x_cluster = [coords[0] for coords in clusters[i]]
        y_cluster = [coords[1] for coords in clusters[i]]
        #z_cluster = [coords[2] for coords in clusters[i]]
        x_mean = sum(x_cluster) / len(x_cluster)
        y_mean = sum(y_cluster) / len(y_cluster)
        #z_mean = sum(z_cluster) / len(z_cluster)
        clusters[i] = [x_mean, y_mean] # z_mean
    return clusters
