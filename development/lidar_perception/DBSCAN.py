# Density-Based Spatial Clustering of Applications with Noise (DBSCAN)

import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN

def init_plot_2D(title, xlabel, ylabel):
    fig = plt.figure()
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)

def plot_grid_2D(object_points):
    init_plot_2D("Clustering Grid", "x", "y")

    x = [coords[0] for coords in object_points]
    y = [coords[1] for coords in object_points]

    plt.plot(x, y, '.', color='red')

def plot_clusters(clusters, noise):
    init_plot_2D("Object Segmentation", "x", "y")

    x_noise = [coords[0] for coords in noise]
    y_noise = [coords[1] for coords in noise]

    plt.plot(x_noise, y_noise, '.', color='red')

    colours = ['g', 'grey', 'm', 'orange']
    print(clusters[0]==clusters[1])
    for i in range(len(clusters)):
        x_cluster = [coords[0] for coords in clusters[i]]
        y_cluster = [coords[1] for coords in clusters[i]]
        plt.plot(x_cluster, y_cluster, '.', color=colours[i % len(colours)])
        x_mean  = sum(x_cluster) / len(x_cluster)
        y_mean  = sum(y_cluster) / len(y_cluster)
        plt.plot(x_mean, y_mean, 'x', color='blue')

def init_DBSCAN(object_points):
    print("--------------------")
    print(object_points)
    plot_grid_2D(object_points)
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
        #point = clustering.components_[i]
        point = object_points[i]
        if label != -1:
            clusters[label].append(point)
        else:
            noise.append(point)

    plot_clusters(clusters, noise)

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
