# Density-Based Spatial Clustering of Applications with Noise (DBSCAN)
# https://towardsdatascience.com/the-5-clustering-algorithms-data-scientists-need-to-know-a36d136ef68
import math
import matplotlib.pyplot as plt
from sklearn import cluster

min_points = 4
epsilon = 0.5

def get_distance():
    pass

def label_points(x_ordered_points):
    for i in range(len(x_ordered_points)):
        x_ordered_points[i].append(0)
    return x_ordered_points

def get_neighbourhood(point_index, x_ordered_labelled):
    global min_points
    neighbours = 0
    neighbourhood = []
    current_point = x_ordered_labelled[point_index]
    current_x = current_point[0]
    index = point_index + 1
    finding_neighbours = True
    while finding_neighbours:
        new_point = x_ordered_labelled[index]
        new_point_x = new_point[0]
        if new_point_x - current_x < epsilon:
            if get_distance(current_point, new_point) < epsilon:
                neighbours += 1
                neighbourhood.append([new_point, index])
            index += 1
            if index >= len(x_ordered_labelled):
                finding_neighbours = False
        else:
            finding_neighbours = False
    if neighbours > min_points:
        current_point[3] = 1 # Visited
        print(neighbourhood, "hi")
        return neighbourhood
    else:
        current_point[3] = 2 # Noise (But still visited)
        if point_index + 2 < len(x_ordered_labelled):
            return get_neighbourhood(point_index + 1, x_ordered_labelled) # Try again (with next point)
        else:
            return neighbourhood
        #x_ordered_labelled.pop(point_index)
        # we might reach end on list of poinst so will need to take care of that

def recursive_cluster(cluster, neighbourhood, x_ordered_labelled):
    print(neighbourhood)
    for i in range(len(neighbourhood)):
        # If not already visited
        if neighbourhood[i][0][3] == 0 and neighbourhood[i][1] < len(x_ordered_labelled) - 1:
            print(neighbourhood[i][1])
            new_neighbourhood = get_neighbourhood(neighbourhood[i][1], x_ordered_labelled)
            cluster.append(new_neighbourhood)
            recursive_cluster(cluster, new_neighbourhood, x_ordered_labelled)
    return cluster

def plot_clusters(clusters, object_points):
    init_plot_2D("Clustering Grid", "x", "y")

    x = [coords[0] for coords in object_points]
    y = [coords[1] for coords in object_points]

    plt.plot(x, y, '.', color='red')

    x_means = []
    y_means = []
    for i in range(len(clusters)):
        print("\n")
        print("Cluster", i)
        x_mean = 0
        y_mean = 0
        for j in range(len(clusters[i][0])):
            x_mean += clusters[i][0][j][0][0]
            y_mean += clusters[i][0][j][0][1]
            print(clusters[i][0][j], "hi")
        x_means.append(x_mean / len(clusters[i][0]))
        y_means.append(y_mean / len(clusters[i][0]))
        print(x_mean, y_mean)

    plt.plot(x_means, y_means, '.', color='blue')

    # sort by x
# then check for all points within epsilon x range
# check their norm against current point
# if all good then add to neighbourhood and continue
def object_clustering(x_ordered_labelled):
    print("Starting object clustering")
    clusters = []
    point_index = 0

    for i in range(len(x_ordered_labelled)):
        print("Running for loop", i)
        if x_ordered_labelled[i][3] == 0:
            point_index = i
            current_point = x_ordered_labelled[point_index]
            init_neighbourhood = [[current_point, point_index]]
            print(init_neighbourhood)
            cluster = recursive_cluster([], init_neighbourhood, x_ordered_labelled)
            if cluster != []:
                clusters.append(cluster)

    print("\n\n\n")
    print(clusters)
    print(len(clusters))

    return clusters

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

def get_distance(point_a, point_b):
    distance = math.sqrt((point_b[0] - point_a[0])**2 + (point_b[1]-point_a[1])**2 + (point_b[2]-point_a[2])**2)
    return distance

def compute_cluster(current_point_index, x_ordered_labelled, clusters):
    latest_cluster = clusters
    cluster_origin_point = x_ordered_labelled[current_point_index]

    for i in range(0, len(clusters[len()])):
        pass



    # need to remove all points in cluster from x_ordered_labelled


def init_cluster_old(x_ordered_labelled, noise, clusters):
    # Step 1
    global epsilon
    global min_points

    current_point_index = 0
    current_point = x_ordered_labelled[current_point_index]
    current_neighbourhood = get_neighbourhood(current_point_index, x_ordered_labelled)

    # Step 2
    if len(current_neighbourhood >= min_points):
        clusters.append([[current_point] + current_neighbourhood])
        for i in range(len(clusters[len(clusters)-1])):
            x_ordered_labelled.pop(0)
        compute_cluster(current_point_index, x_ordered_labelled, clusters)
    else:
        noise.append(x_ordered_labelled[current_point_index])
        x_ordered_labelled.pop(current_point_index)
        init_cluster_old(x_ordered_labelled, noise)

def init_cluster_old(x_ordered_labelled, clusters):
    global min_points
    current_point = x_ordered_labelled[0]
    current_neighbourhood = get_neighbourhood(0, x_ordered_labelled)
    if len(current_neighbourhood) >= min_points:
        # Work here
        new_cluster = compute_cluster([[current_point, 0] + current_neighbourhood], x_ordered_labelled)
        clusters.append(new_cluster)
        pass
    else:
        x_ordered_labelled.pop(0) # if len(x_ordered_labelled) > 0
        init_cluster_old(x_ordered_labelled, clusters)






def get_neighbourhood(origin_index, x_ordered_labelled):
    global epsilon
    origin_point = x_ordered_labelled[origin_index]
    neighbourhood = []
    for i in range(0, len(x_ordered_labelled)):
        if i != origin_index:
            potential_neighbour = x_ordered_labelled[i]
            if potential_neighbour[0] <= epsilon:
                delta_distance = get_distance(origin_point, potential_neighbour)
                if delta_distance < epsilon:
                    neighbourhood.append([potential_neighbour, i])
            else:
                break
    return neighbourhood

def compute_cluster(cluster_origin_index, x_ordered_labelled, cluster):
    global min_points
    origin_neighbourhood = get_neighbourhood(cluster_origin_index, x_ordered_labelled)
    if len(origin_neighbourhood) >= min_points:
        x_ordered_labelled[cluster_origin_index][3] = 1
        cluster.append(origin_neighbourhood)
        for i in range(len(origin_neighbourhood)):
            new_cluster_origin_index = origin_neighbourhood[i][1]
            compute_cluster(new_cluster_origin_index, x_ordered_labelled, cluster)
    else:
        x_ordered_labelled[cluster_origin_index][3] = 2

def init_cluster():    
    clustering = True
    while clustering:
        compute_cluster()
    pass

def init_DBSCAN_old(object_points):
    print("\n\n----------------------------------------------------------------")
    global epsilon
    global min_points
    
    x_ordered_points = sorted(object_points, key = lambda coords: coords[0])
    x_ordered_labelled = label_points(x_ordered_points)

    cluster_origin_index = 0
    empty_cluster = []
    init_cluster(cluster_origin_index, x_ordered_labelled, empty_cluster)
    
from sklearn.cluster import DBSCAN
import numpy as np

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


