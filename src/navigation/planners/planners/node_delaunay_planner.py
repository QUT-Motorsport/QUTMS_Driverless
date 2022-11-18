from math import pi
import time

import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import UnivariateSpline
from scipy.spatial import Delaunay

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from driverless_msgs.msg import Cone, ConeWithCovariance, PathPoint, PathStamped, TrackDetectionStamped
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from driverless_common.marker import delaunay_marker_msg
from driverless_common.shutdown_node import ShutdownNode

from typing import List, Tuple

LEFT_CONE_COLOUR = Cone.BLUE
RIGHT_CONE_COLOUR = Cone.YELLOW

MAX_CONE_DISTANCE = 8
S = 0.5


def is_internal(simplice: np.ndarray, track: np.ndarray) -> bool:
    """
    Determines if a simplice is internal to the track.
    * param simplice: The simplice to check.
    * param track: The track to check against.
    * return: True if the simplice is internal, False otherwise.
    """
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
    """
    Calculates the angle between the vectors v1->v2 and v1->v3.
    * param v1: The first vertex.
    * param v2: The second vertex.
    * param v3: The third vertex.
    * return: The angle between the vectors v1->v2 and v1->v3.
    """
    # compute vectors
    v12 = v2 - v1
    v13 = v3 - v1
    dot = np.dot(v12, v13)
    norm = np.linalg.norm(v12) * np.linalg.norm(v13)
    # compute vector angle
    return np.arccos(dot / norm)


def approximate_b_spline_path(
    x: np.ndarray, y: np.ndarray, n_path_points: int
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    ADAPTED FROM: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/BSplinePath/bspline_path.py \n
    Approximates a B-Spline path through the given points.
    * param x: The x coordinates of the cones.
    * param y: The y coordinates of the cones.
    * param n_path_points: The number of spline points to generate.
    * return: The x, y, heading and curvature of points on the spline.
    """

    distances = calc_distance_vector(x, y)

    spl_i_x = UnivariateSpline(distances, x, k=3, s=S)
    spl_i_y = UnivariateSpline(distances, y, k=3, s=S)

    sampled = np.linspace(0.0, distances[-1], n_path_points)
    return evaluate_spline(sampled, spl_i_x, spl_i_y)


def calc_distance_vector(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """
    Calculates the distance vector between the given points.
    * param x: The x coordinates of the points.
    * param y: The y coordinates of the points.
    * return: The distance vector between the given points.
    """
    dx, dy = np.diff(x), np.diff(y)
    distances = np.cumsum([np.hypot(idx, idy) for idx, idy in zip(dx, dy)])
    distances: np.ndarray = np.concatenate(([0.0], distances))
    distances /= distances[-1]
    return distances


def evaluate_spline(sampled: np.ndarray, spl_i_x: UnivariateSpline, spl_i_y: UnivariateSpline):
    """
    Evaluates the given spline at the given points.
    * param sampled: The points to evaluate the spline at.
    * param spl_i_x: The x spline.
    * param spl_i_y: The y spline.
    * return: The x, y, heading and curvature of points on the spline.
    """
    x = spl_i_x(sampled)
    y = spl_i_y(sampled)
    dx = spl_i_x.derivative(1)(sampled)
    dy = spl_i_y.derivative(1)(sampled)
    heading = np.arctan2(dy, dx)
    ddx = spl_i_x.derivative(2)(sampled)
    ddy = spl_i_y.derivative(2)(sampled)
    curvature = (ddy * dx - ddx * dy) / np.power(dx * dx + dy * dy, 2.0 / 3.0)
    return (
        np.array(x),
        y,
        heading,
        curvature,
    )


# debug img params
cv_bridge = CvBridge()  # translate ROS image messages to OpenCV

HEIGHT = 640
WIDTH = 640

TRACK_WIDTH = 100
TRACK_LENGTH = 50

scale = WIDTH // max(TRACK_WIDTH, TRACK_LENGTH)

img_origin_x = TRACK_WIDTH * scale // 3
img_origin_y = (HEIGHT - TRACK_LENGTH * scale) // 3


class TrackPlanner(Node):
    fig = plt.figure()
    ax = fig.add_subplot()

    def __init__(self):
        super().__init__("global_planner_node")

        # sub to track for all cone locations relative to car start point
        self.create_subscription(TrackDetectionStamped, "/slam/global_map", self.callback, 10)

        # publishers
        self.path_publisher: Publisher = self.create_publisher(PathStamped, "/planner/path", 1)
        self.marker_publisher: Publisher = self.create_publisher(Marker, "/markers/delaunay_lines", 1)
        self.img_publisher: Publisher = self.create_publisher(Image, "/debug_imgs/delaunay_debug_img", 1)

        self.get_logger().info("---Delaunay planner node initalised---")

    def callback(self, track_msg: TrackDetectionStamped):
        self.get_logger().debug("Received track")
        start = time.perf_counter()

        cones_with_cov: List[ConeWithCovariance] = track_msg.cones

        # get left and right cones
        left_cones = [c.cone for c in cones_with_cov if c.cone.color == LEFT_CONE_COLOUR]
        right_cones = [c.cone for c in cones_with_cov if c.cone.color == RIGHT_CONE_COLOUR]

        if len(left_cones) < 2 or len(right_cones) < 2:  # no cones
            return

        # order cones by distance from car
        left_cones.sort(key=lambda c: c.location.x)
        right_cones.sort(key=lambda c: c.location.x)

        # make one array with alternating left and right cones
        cones = left_cones + right_cones
        track = np.array([[c.location.x, c.location.y] for c in cones])

        # # compute delaunay triangulation
        tri = Delaunay(track)
        # only get internal triangles
        internal_triangles = np.array([t for t in tri.simplices if is_internal(t, track)])

        # only take triangles with all sides shorter than MAX_CONE_DISTANCE
        internal_triangles = np.array(
            [
                t
                for t in internal_triangles
                if np.linalg.norm(track[t[0]] - track[t[1]]) < MAX_CONE_DISTANCE
                and np.linalg.norm(track[t[1]] - track[t[2]]) < MAX_CONE_DISTANCE
                and np.linalg.norm(track[t[2]] - track[t[0]]) < MAX_CONE_DISTANCE
            ]
        )

        # get all lines that go to a different coloured cone
        lines: list = []
        disp_lines: list = []
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
                lines.append(np.vstack((v1, v2)))
            if c2 != c3:
                lines.append(np.vstack((v2, v3)))
            if c3 != c1:
                lines.append(np.vstack((v3, v1)))
            disp_lines.append(np.vstack((v1, v2)))
            disp_lines.append(np.vstack((v2, v3)))
            disp_lines.append(np.vstack((v3, v1)))

        # get midpoint of each line
        midpoints = np.array([(l[0] + l[1]) / 2 for l in lines])
        # get unique midpoints
        midpoints = np.unique(midpoints, axis=0)

        # order midpoints by distance to each other
        if midpoints.size == 0:
            return
        ordered_midpoints = [midpoints[0]]
        midpoints = np.delete(midpoints, 0, axis=0)
        while len(midpoints) != 0:
            # get closest midpoint to last ordered midpoint
            closest = np.argmin(np.linalg.norm(midpoints - ordered_midpoints[-1], axis=1))
            # add to ordered midpoints
            ordered_midpoints.append(midpoints[closest])
            # remove from midpoints
            midpoints = np.delete(midpoints, closest, axis=0)
        ordered_midpoints = np.array(ordered_midpoints)

        if len(ordered_midpoints) <= 3:
            headings = np.zeros(len(ordered_midpoints))
            path = np.hstack((ordered_midpoints, headings.reshape(-1, 1)))

        else:
            # interpolate midpoints with a B-Spline
            points = len(ordered_midpoints) * 4
            rix, riy, heading, curvature = approximate_b_spline_path(
                ordered_midpoints[:, 0], ordered_midpoints[:, 1], points
            )
            path = np.vstack((rix, riy, heading)).T

        # publish path
        path_msg: list[PathPoint] = []
        for i in path:
            path_point = PathPoint()
            path_point.location = Point(x=i[0], y=i[1], z=0.0)
            path_point.turn_intensity = 0.0
            path_msg.append(path_point)

        self.path_publisher.publish(PathStamped(path=path_msg))

        # publish delaunay lines
        # make pairs of points for each line
        debug_img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

        delaunay_lines = []
        track_colours = []
        for l in disp_lines:
            delaunay_lines.append(Point(x=l[0][0], y=l[0][1], z=0.0))
            delaunay_lines.append(Point(x=l[1][0], y=l[1][1], z=0.0))
            track_colours.append(ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.2))
            track_colours.append(ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.2))

            cv2.line(
                debug_img,
                (int(l[0][0] * scale + img_origin_x), int(l[0][1] * scale + img_origin_y)),
                (int(l[1][0] * scale + img_origin_x), int(l[1][1] * scale + img_origin_y)),
                (255, 255, 255),
                1,
            )

        for p in path:
            cv2.circle(
                debug_img, (int(p[0] * scale + img_origin_x), int(p[1] * scale + img_origin_y)), 2, (0, 0, 255), -1
            )

        # make marker message
        marker_msg = delaunay_marker_msg(delaunay_lines, track_colours)
        self.marker_publisher.publish(marker_msg)
        self.img_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))

        self.get_logger().debug(f"Planned path in {time.perf_counter() - start}s")

        ## make plot
        #self.ax.clear()
        #self.ax.triplot(track[:, 0], track[:, 1], internal_triangles)
        #for line in lines:
        #    self.ax.plot(line[:, 0], line[:, 1], "g-")
        ## number midpoints
        #for i, p in enumerate(ordered_midpoints):
        #    self.ax.text(p[0], p[1], str(i), fontsize=14)
        #self.ax.plot(ordered_midpoints[:, 0], ordered_midpoints[:, 1], "ro")
        #self.ax.plot(path[:,0], path[:,1], "r-")

        plt.pause(0.03)
        plt.show(block=False)


def main(args=None):
    rclpy.init(args=args)
    node = TrackPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
