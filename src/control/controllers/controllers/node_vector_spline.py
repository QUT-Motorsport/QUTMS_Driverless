from math import atan2, pi
import time

from colour import Color
import cv2
import numpy as np
from scipy.interpolate import UnivariateSpline
from scipy.spatial import distance

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import Cone, ConeDetectionStamped
from sensor_msgs.msg import Image

from driverless_common.common import QOS_ALL
from driverless_common.draw import draw_markers, draw_steering, loc_to_img_pt
from driverless_common.point import Point, cone_to_point, dist

from typing import List, Optional, Tuple

cv_bridge = CvBridge()  # translate ROS image messages to OpenCV

red = Color("red")
blue = Color("blue")
col_range = list(blue.range_to(red, 100))

Colour = Tuple[int, int, int]

ORIGIN = Point(0, 0)

LEFT_CONE_COLOUR = Cone.BLUE
RIGHT_CONE_COLOUR = Cone.YELLOW

left_mid_col = (245, 215, 66)
right_mid_col = (66, 194, 245)

LOOKAHEAD = 2  # m


def approximate_b_spline_path(
    x: np.ndarray, y: np.ndarray, n_path_points: int, order: int
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

    spl_i_x = UnivariateSpline(distances, x, k=order)
    spl_i_y = UnivariateSpline(distances, y, k=order)

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


def evaluate_spline(
    sampled: np.ndarray, spl_i_x: UnivariateSpline, spl_i_y: UnivariateSpline
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
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


def get_RVWP(path: np.ndarray, lookahead: float) -> np.ndarray:
    """
    Retrieve angle between two points
    * param car_pos: [x,y] coords of point 1
    * param path: [[x0,y0,i0],[x1,y1,i1],...,[xn-1,yn-1,in-1]] path points
    * param rvwp_lookahead: distance to look ahead for the RVWP
    * return: RVWP position as [x,y,i]
    """
    dists: np.ndarray = distance.cdist(
        path,  # search all path points for x,y cols up to 3rd col (intensity)
        [[0, 0]],
        "euclidean",
    )

    # find the index of the point that is closest to the lookahead distance
    idx = np.argmin(np.abs(dists - lookahead))
    return path[idx]


class VectorReactiveController(Node):
    Kp_ang: float
    target_vel: float
    target_accel: float
    in_dist: float  # m
    pub_accel: bool
    ebs_test: bool
    last_time: float = 0

    def __init__(self):
        super().__init__("reactive_vector_controller_node")

        self.ebs_test = self.declare_parameter("ebs_control", False).get_parameter_value().bool_value
        self.get_logger().info("EBS Control: " + str(self.ebs_test))

        if self.ebs_test:
            self.Kp_ang = -1.0
            self.target_vel = 12.0  # m/s
            self.target_accel = 0.0
            self.in_dist = 2.0
            self.pub_accel = False

            self.create_subscription(ConeDetectionStamped, "/vision/cone_detection", self.callback, QOS_ALL)
        else:
            self.Kp_ang = -3.0
            self.target_vel = 3.0  # m/s
            self.target_accel = 0.0
            self.in_dist = 2.0
            self.pub_accel = False
            # self.create_subscription(ConeDetectionStamped, "/slam/local_map", self.callback, 1)
            self.create_subscription(ConeDetectionStamped, "/vision/cone_detection", self.callback, QOS_ALL)

        self.control_publisher: Publisher = self.create_publisher(AckermannDriveStamped, "/control/driving_command", 1)
        self.accel_publisher: Publisher = self.create_publisher(AckermannDriveStamped, "/control/accel_command", 1)

        self.vector_publisher: Publisher = self.create_publisher(Image, "/debug_imgs/vector_reactive_img", 1)

        self.get_logger().info("---Vector spline controller node initalised---")

    def callback(self, cone_msg: ConeDetectionStamped):
        self.get_logger().debug("Received detection")

        # safety critical, set to 0 if not good detection
        speed = 0.0
        steering_angle = 0.0

        cones: List[Cone] = cone_msg.cones

        if cone_msg.header.frame_id == "velodyne":
            # change colour of cones based on on y pos as this is lidar scan
            for i in range(len(cones)):
                if cones[i].location.y > 0:
                    cones[i].color = LEFT_CONE_COLOUR
                else:
                    cones[i].color = RIGHT_CONE_COLOUR

        left_cones = [c for c in cones if c.color == LEFT_CONE_COLOUR and abs(c.location.y) < 8]
        right_cones = [c for c in cones if c.color == RIGHT_CONE_COLOUR and abs(c.location.y) < 8]
        orange_cones = [c for c in cones if c.color == Cone.ORANGE_BIG and abs(c.location.y) < 8]

        # add orange cones to left and right arrays
        orange_avg = np.mean([c.location.y for c in orange_cones])
        for c in orange_cones:
            if c.location.y > orange_avg:
                left_cones.append(c)
            else:
                right_cones.append(c)

        closest_left: Optional[Cone] = None
        closest_right: Optional[Cone] = None

        debug_img = draw_markers(cones)

        # find points from left side
        midpoints: List[Point] = []

        if len(left_cones) > 0:
            closest_left = sorted(left_cones, key=lambda c: dist(ORIGIN, cone_to_point(c)))

            for i in range(0, len(closest_left) - 1):
                p1 = cone_to_point(closest_left[i])
                p2 = cone_to_point(closest_left[i + 1])

                cv2.line(
                    debug_img,
                    loc_to_img_pt(p1.x, p1.y).to_tuple(),
                    loc_to_img_pt(p2.x, p2.y).to_tuple(),
                    left_mid_col,
                    thickness=2,
                )

                p1.x += 0.001
                diff = p2 - p1  # temp solution, adding pertubation to divide by zero BAD SOLUTION FIX LATER
                mult = 1.0 / (dist(ORIGIN, diff))
                Rtemp = diff * mult

                R = Rtemp * self.in_dist
                temp = R * 1.0
                temp.x = R.y
                temp.y = -R.x

                self.get_logger().debug(f"{diff}, {Rtemp},{R}, {mult}")

                midSeg = p1 + (diff * 0.5)
                midPoint = midSeg + temp
                midpoints.append(midPoint)

                cv2.line(
                    debug_img,
                    loc_to_img_pt(midSeg.x, midSeg.y).to_tuple(),
                    loc_to_img_pt(midPoint.x, midPoint.y).to_tuple(),
                    (0, 255, 0),
                    thickness=2,
                )

                cv2.drawMarker(
                    debug_img,
                    loc_to_img_pt(midPoint.x, midPoint.y).to_tuple(),
                    left_mid_col,
                    markerType=cv2.MARKER_TRIANGLE_UP,
                    markerSize=5,
                    thickness=5,
                )

        if len(right_cones) > 0:
            closest_right = sorted(right_cones, key=lambda c: dist(ORIGIN, cone_to_point(c)))

            for i in range(0, len(closest_right) - 1):
                p1 = cone_to_point(closest_right[i])
                p2 = cone_to_point(closest_right[i + 1])

                cv2.line(
                    debug_img,
                    loc_to_img_pt(p1.x, p1.y).to_tuple(),
                    loc_to_img_pt(p2.x, p2.y).to_tuple(),
                    right_mid_col,
                    thickness=2,
                )

                p1.x += 0.001
                diff = p2 - p1  # temp solution, adding pertubation to divide by zero BAD SOLUTION FIX LATER
                mult = 1.0 / (dist(ORIGIN, diff))
                Rtemp = diff * mult

                R = Rtemp * self.in_dist
                temp = R * 1.0
                temp.x = -R.y
                temp.y = R.x

                self.get_logger().debug(f"{diff}, {Rtemp},{R}, {mult}")

                midSeg = p1 + (diff * 0.5)
                midPoint = midSeg + temp
                midpoints.append(midPoint)

                cv2.line(
                    debug_img,
                    loc_to_img_pt(midSeg.x, midSeg.y).to_tuple(),
                    loc_to_img_pt(midPoint.x, midPoint.y).to_tuple(),
                    (125, 125, 0),
                    thickness=2,
                )

                cv2.drawMarker(
                    debug_img,
                    loc_to_img_pt(midPoint.x, midPoint.y).to_tuple(),
                    right_mid_col,
                    markerType=cv2.MARKER_TRIANGLE_UP,
                    markerSize=5,
                    thickness=2,
                )

        self.get_logger().debug(f"Midpoints: {midpoints}")

        midpoints.append(ORIGIN)
        # adding points behind the car to fix axis
        midpoints.append(Point(-1, 0))
        midpoints.append(Point(-2, 0))
        sorted_midpoints = sorted(midpoints, key=lambda c: dist(ORIGIN, c))
        midpoints = []

        i = 0
        while i < len(sorted_midpoints):
            midpoint = sorted_midpoints[i]
            for j in range(i + 1, len(sorted_midpoints)):
                if dist(midpoint, sorted_midpoints[j]) < 2:
                    sum_ = midpoint + sorted_midpoints[j]
                    midpoint = Point(sum_.x / 2, sum_.y / 2)
                else:
                    i = j - 1
                    break
            midpoints.append(midpoint)
            i += 1

        for p in midpoints:
            cv2.drawMarker(
                debug_img,
                loc_to_img_pt(p.x, p.y).to_tuple(),
                (0, 0, 255),
                markerType=cv2.MARKER_TRIANGLE_UP,
                markerSize=20,
                thickness=2,
            )

        # if we found midpoints
        if len(midpoints) > 2:
            orderSpline = len(midpoints) - 1
            if orderSpline > 3:
                orderSpline = 3

            orderSpline = 2  # track hard code linear quick fix

            x = [p.x for p in midpoints]
            y = [p.y for p in midpoints]

            splineResult = approximate_b_spline_path(x, y, 15, orderSpline)
            splineX = splineResult[0]
            splineY = splineResult[1]

            self.get_logger().debug(f"Spline {splineX} {splineY}")

            for i in range(0, len(splineX) - 1):
                cv2.line(
                    debug_img,
                    loc_to_img_pt(splineX[i], splineY[i]).to_tuple(),
                    loc_to_img_pt(splineX[i + 1], splineY[i + 1]).to_tuple(),
                    (255, 0, 0),
                    thickness=2,
                )

            spline = np.concatenate((splineX.reshape(-1, 1), splineY.reshape(-1, 1)), axis=1)
            rvwp = get_RVWP(spline, LOOKAHEAD)

            self.get_logger().debug(f"Target: {rvwp}")
            steering_angle = np.degrees(atan2(rvwp[0], rvwp[1]) - pi / 2)

            cv2.drawMarker(
                debug_img,
                loc_to_img_pt(rvwp[0], rvwp[1]).to_tuple(),
                (0, 255, 0),
                markerType=cv2.MARKER_TRIANGLE_UP,
                markerSize=5,
                thickness=2,
            )

            speed = self.target_vel
            debug_img = draw_steering(debug_img, steering_angle, speed)  # draw steering angle and vel data on image

        # publish message
        control_msg = AckermannDriveStamped()
        control_msg.header.stamp = cone_msg.header.stamp
        control_msg.drive.steering_angle = steering_angle * self.Kp_ang
        control_msg.drive.speed = speed
        control_msg.drive.acceleration = self.target_accel

        self.control_publisher.publish(control_msg)
        if self.pub_accel:
            self.accel_publisher.publish(control_msg)

        self.vector_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))

        self.get_logger().debug(f"Time: {time.time() - self.last_time}")
        self.last_time = time.time()


def main(args=None):
    rclpy.init(args=args)
    node = VectorReactiveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
