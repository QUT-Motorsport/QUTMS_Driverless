"""

Object clustering with k-means algorithm

author: Atsushi Sakai (@Atsushi_twi)

"""

import math
import matplotlib.pyplot as plt
import random

# k means parameters
MAX_LOOP = 10
DCOST_TH = 0.1
show_animation = True


def kmeans_clustering(rx, ry, nc):
    clusters = Clusters(rx, ry, nc)
    clusters.calc_centroid()

    pre_cost = float("inf")
    for loop in range(MAX_LOOP):
        cost = clusters.update_clusters()
        clusters.calc_centroid()

        d_cost = abs(cost - pre_cost)
        if d_cost < DCOST_TH:
            break
        pre_cost = cost

    return clusters


class Clusters:

    def __init__(self, x, y, n_label):
        self.x = x
        self.y = y
        self.n_data = len(self.x)
        self.n_label = n_label
        self.labels = [random.randint(0, n_label - 1)
                       for _ in range(self.n_data)]
        self.center_x = [0.0 for _ in range(n_label)]
        self.center_y = [0.0 for _ in range(n_label)]

    def plot_cluster(self):
        labs = []
        for label in set(self.labels):
            x, y = self._get_labeled_x_y(label)

            labs.append([x,y])
        return labs
    
    def get_cent(self):
        cents = []
        for label in set(self.labels):
            if abs(self.center_x[label]) < 250 and abs(self.center_y[label]) < 250:
                cents.append([self.center_x[label], self.center_y[label]])
        return cents

    def calc_centroid(self):
        for label in set(self.labels):
            x, y = self._get_labeled_x_y(label)
            n_data = len(x)
            self.center_x[label] = sum(x) / n_data
            self.center_y[label] = sum(y) / n_data

    def update_clusters(self):
        cost = 0.0

        for ip in range(self.n_data):
            px = self.x[ip]
            py = self.y[ip]

            dx = [icx - px for icx in self.center_x]
            dy = [icy - py for icy in self.center_y]

            dist_list = [math.hypot(idx, idy) for (idx, idy) in zip(dx, dy)]
            min_dist = min(dist_list)
            min_id = dist_list.index(min_dist)
            self.labels[ip] = min_id
            cost += min_dist

        return cost

    def _get_labeled_x_y(self, target_label):
        x = [self.x[i] for i, label in enumerate(self.labels) if label == target_label]
        y = [self.y[i] for i, label in enumerate(self.labels) if label == target_label]
        return x, y



