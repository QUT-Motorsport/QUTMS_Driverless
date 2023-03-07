from math import atan, atan2, degrees, floor, pi, sqrt
import time

import cv2
import numpy as np

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import Cone, ConeDetectionStamped
from sensor_msgs.msg import Image

from driverless_common.draw import *
from driverless_common.point import Point
from driverless_common.shutdown_node import ShutdownNode

from typing import List, Tuple

Colour = Tuple[int, int, int]

# translate ROS image messages to OpenCV
cv_bridge = CvBridge()

ORIGIN = Point(0, 0)

LEFT_CONE_COLOUR = Cone.BLUE
RIGHT_CONE_COLOUR = Cone.YELLOW

WEIGHT = -1

POINTS_PER_LINE = 10
SPLINES = 11
if SPLINES % 2 == 0:
    SPLINES += 1


class PoinntFitController(Node):
    prev_steering_angle: float = 0
    Kp_angle: float = 20
    targ_vel: float = 4  # m/s
    debug_img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)  # create black image

    steeringPaths: List[List[Point]] = []
    line_len: float = 15.0  # len of spline line

    def __init__(self):
        super().__init__("point_fit_controller_node")

        # debug image
        self.create_subscription(Image, "/debug_imgs/lidar_det_img", self.img_callback, 1)
        # cone detections
        self.create_subscription(ConeDetectionStamped, "/lidar/cone_detection", self.callback, 1)

        # publishers
        self.control_publisher: Publisher = self.create_publisher(AckermannDriveStamped, "/control/driving_command", 1)

        self.debug_img_publisher: Publisher = self.create_publisher(Image, "/debug_imgs/control_img", 1)

        self.initialize_splines()  # initialize steering curves to take

        self.get_logger().info("---Point Fitting Controller Node Initalised---")

    def initialize_splines(self):
        # cicle will pass through 0,0 and and additional point:
        # for each end point, determine the circle defining it with 10 points
        end_points = []
        end_points.append(Point(self.line_len / 2, -5 * self.line_len / 10))  # origin
        end_points.append(Point(self.line_len / 2 + self.line_len / 10, -4 * self.line_len / 10))
        end_points.append(Point(self.line_len / 2 + 2 * self.line_len / 10, -3 * self.line_len / 10))
        end_points.append(Point(self.line_len / 2 + 3 * self.line_len / 10, -2 * self.line_len / 10))
        end_points.append(Point(self.line_len / 2 + 4 * self.line_len / 10, -1 * self.line_len / 10))
        end_points.append(Point(self.line_len, 0.0))
        end_points.append(Point(self.line_len / 2 + 4 * self.line_len / 10, 1 * self.line_len / 10))
        end_points.append(Point(self.line_len / 2 + 3 * self.line_len / 10, 2 * self.line_len / 10))
        end_points.append(Point(self.line_len / 2 + 2 * self.line_len / 10, 3 * self.line_len / 10))
        end_points.append(Point(self.line_len / 2 + self.line_len / 10, 4 * self.line_len / 10))
        end_points.append(Point(self.line_len / 2, 5 * self.line_len / 10))

        self.steering_paths = [[] for i in range(SPLINES)]  # create empty list of lists
        for index in range(SPLINES):
            self.steering_paths[index] = []

            # straight up
            if index == int(SPLINES / 2):
                x = 0
                y = 0
                self.steering_paths[int(SPLINES / 2)] = []
                for i in range(POINTS_PER_LINE):
                    self.steering_paths[int(SPLINES / 2)].append(Point(x, y))  # store
                    x = x + self.line_len / 9
                continue

            h = (end_points[index].x ** 2 + end_points[index].y ** 2) / (2 * end_points[index].y)  # center
            r = abs(h)  # radius
            circum = pi * 2 * r  # circumference
            fraction = self.line_len / circum  # percentage of circle
            angle = 2 * pi * fraction / 10.0  # angle / 10 in radians

            # if the circle is to the right of the origin, the angle needs to be negative
            if index > int(SPLINES / 2):
                angle = -angle
                r = -r

            center = Point(0, h)  # center of circle
            vector = Point(0, r)  # vector to point
            for i in range(POINTS_PER_LINE):
                self.steering_paths[index].append(Point(-(vector.x + center.x), vector.y + center.y))  # store
                vector = Point(
                    cos(angle) * vector.x - sin(angle) * vector.y, sin(angle) * vector.x + cos(angle) * vector.y
                )  # rotate

    def img_callback(self, img_msg: Image):
        self.debug_img = cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")

    def callback(self, cone_msg: ConeDetectionStamped):
        self.get_logger().debug("Received detection")
        start: float = time.perf_counter()  # begin a timer

        # safety critical, set to 0 if not good detection
        speed = 0.0
        steering_angle = 0.0

        cones: List[Cone] = [
            c for c in cone_msg.cones if abs(c.location.y) < 7.0 and c.location.x < 12
        ]  # only cones in front of car

        best_spline = 0
        if len(cones) != 0:
            s_error = [0] * len(self.steering_paths)  # error for each spline
            max_error = 1e20

            for n_spline, steering_path in enumerate(self.steering_paths):
                n_spline_points = len(steering_path)
                for cone in cones:
                    # distance from origin
                    distance = sqrt(cone.location.x**2 + cone.location.y**2)

                    # bin
                    bin = min(n_spline_points - 1, int(round(distance / self.line_len * n_spline_points, 0)) - 1)
                    if bin < 0:
                        bin = 0

                    s_error[n_spline] += sqrt(
                        (cone.location.x - steering_path[bin].x) ** 2 + (cone.location.y - steering_path[bin].y) ** 2
                    )

                if s_error[n_spline] < max_error:
                    max_error = s_error[n_spline]
                    best_spline = n_spline

                for point in steering_path:
                    # draw each element in target spline
                    cv2.drawMarker(
                        self.debug_img,
                        loc_to_img_pt(point.x, point.y).to_tuple(),
                        (0, 0, 255),
                        markerType=cv2.MARKER_SQUARE,
                        markerSize=2,
                        thickness=2,
                    )

            best_path = self.steering_paths[best_spline]
            for point in best_path:
                # draw each element in target spline
                cv2.drawMarker(
                    self.debug_img,
                    loc_to_img_pt(point.x, point.y).to_tuple(),
                    (0, 255, 0),
                    markerType=cv2.MARKER_SQUARE,
                    markerSize=2,
                    thickness=2,
                )
            self.debug_img_publisher.publish(cv_bridge.cv2_to_imgmsg(self.debug_img, encoding="bgr8"))

            speed = self.targ_vel

        # publish message
        control_msg = AckermannDriveStamped()
        control_msg.header.stamp = cone_msg.header.stamp
        control_msg.drive.steering_angle = degrees(steering_angle)
        control_msg.drive.speed = float(speed)
        self.control_publisher.publish(control_msg)

        self.get_logger().debug(f"Total Time: {str(time.perf_counter() - start)}\n")  # log time


def main(args=None):
    rclpy.init(args=args)
    node = PoinntFitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
