from math import atan, atan2, degrees, pi, sqrt
import time

import cv2
import numpy as np

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import Cone, ConeDetectionStamped, Reset
from sensor_msgs.msg import Image

from driverless_common.draw import *
from driverless_common.point import Point, cone_to_point, dist

from typing import List, Tuple

Colour = Tuple[int, int, int]

# translate ROS image messages to OpenCV
cv_bridge = CvBridge()

ORIGIN = Point(0, 0)

LEFT_CONE_COLOUR = Cone.BLUE
RIGHT_CONE_COLOUR = Cone.YELLOW

WEIGHT = -1


def closest_point_on_curve(point: Point, curve: np.ndarray) -> Point:
    """
    Find the closest point on the curve to the cone
    * param point: the cone coord
    * param curve: the curve
    * return: the closest point on the curve to the cone
    """
    closest_point = ORIGIN
    min_dist = float("inf")
    for i in range(len(curve)):
        p = Point(i / 10, curve[i])
        d = dist(point, p)
        if d < min_dist:
            min_dist = d
            closest_point = p
    return closest_point


class BetterReactiveController(Node):
    prev_steering_angle: float = 0
    Kp_angle: float = 20
    targ_vel: float = 4  # m/s
    r2d: bool = True  # for reset
    debug_img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)  # create black image

    def __init__(self):
        super().__init__("better_reactive_controller")

        # debug image
        self.create_subscription(Image, "/debug_imgs/vision_det_img", self.img_callback, 1)
        # cone detections
        self.create_subscription(ConeDetectionStamped, "/slam/local", self.callback, 1)
        # sim
        # self.create_subscription(ConeDetectionStamped, "/vision/cone_detection", self.callback, 1)

        self.reset_sub = self.create_subscription(Reset, "/reset", self.reset_callback, 10)

        # publishers
        self.control_publisher: Publisher = self.create_publisher(AckermannDriveStamped, "/driving_command", 1)

        self.debug_img_publisher: Publisher = self.create_publisher(Image, "/debug_imgs/control_img", 1)

        self.get_logger().info("---Better Reactive Controller Node Initalised---")
        self.get_logger().info("---Awaing Ready to Drive command *OVERRIDDEN*---")

    def img_callback(self, img_msg: Image):
        self.debug_img = cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")

    def reset_callback(self, msg: Reset):
        self.prev_steering_angle = 0
        self.r2d = True

    def callback(self, cone_msg: ConeDetectionStamped):
        self.get_logger().debug("Received detection")
        start: float = time.perf_counter()  # begin a timer
        if not self.r2d:
            return

        # safety critical, set to 0 if not good detection
        speed = 0.0
        steering_angle = 0.0

        cones: List[Cone] = cone_msg.cones
        left_cones = [c for c in cones if c.color == LEFT_CONE_COLOUR]
        right_cones = [c for c in cones if c.color == RIGHT_CONE_COLOUR]

        # compute 10 quadratic curves based on different steering angles
        # find the best curve that reduces the error from curve to each cone
        # use the steering angle of that curve
        A: np.ndarray = np.linspace(-0.1, 0.1, 15)
        x: np.ndarray = np.linspace(0, 10, 100)
        curves: List[np.ndarray] = []
        # compute the error for each curve
        errors: List[float] = []
        for a in A:
            y = a * x**2
            curves.append(y)

            error = 0
            for cone in left_cones:
                # find the closest point on the curve to the cone
                closest_point: Point = closest_point_on_curve(cone_to_point(cone), y)
                # compute the error
                error += dist(cone_to_point(cone), closest_point) * WEIGHT / len(left_cones)

            for cone in right_cones:
                # find the closest point on the curve to the cone
                closest_point: Point = closest_point_on_curve(cone_to_point(cone), y)
                # compute the error
                error += dist(cone_to_point(cone), closest_point) * -WEIGHT / len(right_cones)

            errors.append(error)

            for i in range(len(x)):
                # draw each element in target spline
                cv2.drawMarker(
                    self.debug_img,
                    loc_to_img_pt(x[i], y[i]).to_tuple(),
                    (0, 0, 255),
                    markerType=cv2.MARKER_SQUARE,
                    markerSize=1,
                    thickness=2,
                )

        # find the best curve
        best_curve: np.ndarray = curves[np.abs(errors).argmin()]
        for i in range(len(x)):
            # draw each element in target spline
            cv2.drawMarker(
                self.debug_img,
                loc_to_img_pt(x[i], best_curve[i]).to_tuple(),
                (0, 255, 0),
                markerType=cv2.MARKER_SQUARE,
                markerSize=1,
                thickness=2,
            )
        self.debug_img_publisher.publish(cv_bridge.cv2_to_imgmsg(self.debug_img, encoding="bgr8"))

        best_curve_steering_angle: float = A[np.abs(errors).argmin()] * self.Kp_angle

        # publish message
        control_msg = AckermannDriveStamped()
        control_msg.drive.steering_angle = degrees(best_curve_steering_angle)
        control_msg.drive.speed = self.targ_vel
        self.control_publisher.publish(control_msg)

        self.get_logger().debug(f"Total Time: {str(time.perf_counter() - start)}\n")  # log time


def main(args=None):
    rclpy.init(args=args)
    node = BetterReactiveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
