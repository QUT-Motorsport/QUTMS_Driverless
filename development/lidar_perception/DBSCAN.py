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

def plot_clusters_old(clusters, object_points):
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
