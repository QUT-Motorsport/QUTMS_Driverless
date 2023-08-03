import numpy as np
from sklearn.neighbors import KDTree

import rclpy

from driverless_common.common import angle, wrap_to_pi

from .node_pure_pursuit import PurePursuit

from typing import List


class PurePursuitKdtree(PurePursuit):
    def __init__(self):
        super().__init__("fast_pure_pursuit_node")
        self.get_logger().info("---Pure Pursuit Kdtree Node Initalised---")

    def get_rvwp(self, car_pos: List[float]):
        """
        Retrieve angle between two points
        * param car_pos: [x,y,theta] pose of the car
        * param path: [[x0,y0,i0],[x1,y1,i1],...,[xn-1,yn-1,in-1]] path points
        * param rvwp_lookahead: distance to look ahead for the RVWP
        * return: RVWP position as [x,y,i]
        """
        path_xy = [[p[0], p[1]] for p in self.path]

        # find the closest point on the path to the car
        kdtree = KDTree(path_xy)
        close_index = kdtree.query([[car_pos[0], car_pos[1]]], return_distance=False)[0][0]
        close = path_xy[close_index] if close_index is not None else car_pos
        if close_index is None:
            self.get_logger().warn("Could not find closest point, have used car's axle pos")

        # find the first point on the path that is further than the lookahead distance
        # Tragically, there is no way to set a minimum search distance, so I'm giving it the lookahead doubled, we'll
        # receive everything under that distance and filter out the items under the lookahead distance below.
        # Problem there is if there are no points under the double lookahead distance, we're in trouble.
        indexes_raw, distances_raw = kdtree.query_radius(
            [close], r=self.lookahead * 2, return_distance=True, sort_results=True
        )
        indexes_raw, distances_raw = indexes_raw[0], distances_raw[0]

        indexes, distances = [], []
        for i in range(len(indexes_raw)):
            if distances_raw[i] < self.lookahead:
                continue
            # Distances are sorted, so once we get here just grab everything.
            indexes = indexes_raw[i:]
            distances = distances_raw[i:]
            break

        rvwp_dist = float("inf")
        rvwp_index = None
        for i in range(len(indexes)):
            index = indexes[i]
            p = path_xy[index]
            d = distances[i]

            # get angle to check if the point is in front of the car
            ang = angle(close, p)
            error = wrap_to_pi(car_pos[2] - ang)
            if np.pi / 2 > error > -np.pi / 2 and d < rvwp_dist:
                rvwp_dist = d
                rvwp_index = index

        if rvwp_index is None or rvwp_index == close_index:
            self.get_logger().warn("No valid RVWP found, using fallback point")
            path_points_count = len(self.path) - 1
            fallback_point = close_index + self.fallback_path_points_offset
            if fallback_point > path_points_count:
                rvwp_index = abs(path_points_count - fallback_point)
            else:
                rvwp_index = fallback_point

        return self.path[rvwp_index]


def main(args=None):  # begin ros node
    rclpy.init(args=args)
    node = PurePursuitKdtree()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
