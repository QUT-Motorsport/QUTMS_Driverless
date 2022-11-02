from math import pi

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import Delaunay

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from driverless_msgs.msg import Cone, ConeWithCovariance, PathPoint, PathStamped, TrackDetectionStamped

from typing import List

LEFT_CONE_COLOUR = Cone.BLUE
RIGHT_CONE_COLOUR = Cone.YELLOW

MAX_CONE_DISTANCE = 10


def is_internal(simplice: np.ndarray, track: np.ndarray) -> bool:
    """Returns True if the triangle is internal to the track, False otherwise."""
    # get triangle vertices
    v1 = track[simplice[0]]
    v2 = track[simplice[1]]
    v3 = track[simplice[2]]
    # compute angles
    a1 = angle(v1, v2, v3)
    a2 = angle(v2, v3, v1)
    a3 = angle(v3, v1, v2)
    # check if any are external
    if a1 > 2 * pi / 3 or a2 > 2 * pi / 3 or a3 > 2 * pi / 3:
        return False
    return True


def angle(v1: np.ndarray, v2: np.ndarray, v3: np.ndarray) -> float:
    """Returns the angle between the vectors v1->v2 and v1->v3."""
    # compute vectors
    v12 = v2 - v1
    v13 = v3 - v1
    # compute dot product
    dot = np.dot(v12, v13)
    # compute norm
    norm = np.linalg.norm(v12) * np.linalg.norm(v13)
    # compute angle
    return np.arccos(dot / norm)


class TrackPlanner(Node):
    fig = plt.figure()
    ax = fig.add_subplot()

    def __init__(self):
        super().__init__("track_planner")

        # sub to track for all cone locations relative to car start point
        self.create_subscription(TrackDetectionStamped, "/slam/local", self.callback, 10)

        # publishers
        self.path_publisher: Publisher = self.create_publisher(PathStamped, "/planner/path", 1)

        self.get_logger().info("---Delaunay Planner Node Initalised---")

    def callback(self, track_msg: TrackDetectionStamped):
        self.get_logger().debug("Received track")

        cones_with_cov: List[ConeWithCovariance] = track_msg.cones

        # get left and right cones
        left_cones = [c.cone for c in cones_with_cov if c.cone.color == LEFT_CONE_COLOUR]
        right_cones = [c.cone for c in cones_with_cov if c.cone.color == RIGHT_CONE_COLOUR]

        # order cones by distance from car
        left_cones.sort(key=lambda c: c.location.x)
        right_cones.sort(key=lambda c: c.location.x)

        # make one array with alternating left and right cones
        cones = left_cones + right_cones
        track = np.array([[c.location.x, c.location.y] for c in cones])

        # # compute delaunay triangulation
        tri = Delaunay(track)
        # only get internal triangles
        internal_triangles = [t for t in tri.simplices if is_internal(t, track)]

        # only take triangles with all sides shorter than MAX_CONE_DISTANCE
        internal_triangles = [
            t
            for t in internal_triangles
            if np.linalg.norm(track[t[0]] - track[t[1]]) < MAX_CONE_DISTANCE
            and np.linalg.norm(track[t[1]] - track[t[2]]) < MAX_CONE_DISTANCE
            and np.linalg.norm(track[t[2]] - track[t[0]]) < MAX_CONE_DISTANCE
        ]

        # get all lines that go to a different coloured cone
        lines = []
        for t in internal_triangles:
            # get triangle vertices
            v1 = track[t[0]]
            v2 = track[t[1]]
            v3 = track[t[2]]
            # get cone colours
            c1 = cones[t[0]].color
            c2 = cones[t[1]].color
            c3 = cones[t[2]].color
            # check if any are different
            if c1 != c2:
                lines.append([v1, v2])
            if c2 != c3:
                lines.append([v2, v3])
            if c3 != c1:
                lines.append([v3, v1])
        lines = np.array(lines)

        # make plot
        self.ax.clear()
        self.ax.triplot(track[:, 0], track[:, 1], internal_triangles)
        self.ax.plot(track[:, 0], track[:, 1], "or", markersize=6)
        self.ax.plot(lines[:, :, 0], lines[:, :, 1], "g", linewidth=2)

        plt.pause(0.05)
        plt.show(block=False)


def main(args=None):
    # begin ros node
    rclpy.init(args=args)
    node = TrackPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
