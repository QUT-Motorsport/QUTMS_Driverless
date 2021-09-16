# Density-Based Spatial Clustering of Applications with Noise (DBSCAN)
# https://towardsdatascience.com/the-5-clustering-algorithms-data-scientists-need-to-know-a36d136ef68
import math
import matplotlib.pyplot as plt
from sklearn import cluster

def label_points(x_ordered_points):
    for i in range(len(x_ordered_points)):
        x_ordered_points[i].append(0)
    return x_ordered_points
