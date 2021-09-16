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

