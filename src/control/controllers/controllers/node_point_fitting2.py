from math import cos, pi, sin, sqrt
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

LIDAR_DIST = 1.65  # distance from lidar to front of car
Y_CROP = 6
X_CROP = 7
LINE_LEN = 10.0  # len of spline line
STEERING_VALS = np.linspace(-90, 90, SPLINES)  # steering values to take


class PointFitController(Node):
    targ_vel: float = 1.5  # m/s
    debug_img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)  # create black image

    steering_paths: List[List[Point]] = []
    last_steering_index: int = int(SPLINES / 2)  # index of last steering path takens
    last_counter: int = 0  # counter for last steering path taken

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

        self.get_logger().info("---Trajectory fitting controller initalised---")

    def initialize_splines(self):
        # cicle will pass through 0,0 and and additional point:
        # for each end point, determine the circle defining it with 10 points
        end_points = []
        end_points.append(Point(LINE_LEN / 2 + 1 * LINE_LEN / 10, -5 * LINE_LEN / 10))  # origin
        end_points.append(Point(LINE_LEN / 2 + 2 * LINE_LEN / 10, -4 * LINE_LEN / 10))
        end_points.append(Point(LINE_LEN / 2 + 3 * LINE_LEN / 10, -3 * LINE_LEN / 10))
        end_points.append(Point(LINE_LEN / 2 + 4 * LINE_LEN / 10, -2 * LINE_LEN / 10))
        end_points.append(Point(LINE_LEN / 2 + 5 * LINE_LEN / 10, -1 * LINE_LEN / 10))
        end_points.append(Point(LINE_LEN, 0.0))
        end_points.append(Point(LINE_LEN / 2 + 5 * LINE_LEN / 10, 1 * LINE_LEN / 10))
        end_points.append(Point(LINE_LEN / 2 + 4 * LINE_LEN / 10, 2 * LINE_LEN / 10))
        end_points.append(Point(LINE_LEN / 2 + 3 * LINE_LEN / 10, 3 * LINE_LEN / 10))
        end_points.append(Point(LINE_LEN / 2 + 2 * LINE_LEN / 10, 4 * LINE_LEN / 10))
        end_points.append(Point(LINE_LEN / 2 + 1 * LINE_LEN / 10, 5 * LINE_LEN / 10))

        self.steering_paths = [[] for i in range(SPLINES)]  # create empty list of lists
        for index in range(SPLINES):
            self.steering_paths[index] = []

            # straight up
            if index == int(SPLINES / 2):
                x = -2
                y = 0
                self.steering_paths[int(SPLINES / 2)] = []
                for i in range(POINTS_PER_LINE):
                    self.steering_paths[int(SPLINES / 2)].append(Point(x, y))  # store
                    x = x + LINE_LEN / 9
                continue

            h = (end_points[index].x ** 2 + end_points[index].y ** 2) / (2 * end_points[index].y)  # center
            r = abs(h)  # radius
            circum = pi * 2 * r  # circumference
            fraction = LINE_LEN / circum  # percentage of circle
            angle = 2 * pi * fraction / 10.0  # angle / 10 in radians

            # if the circle is to the right of the origin, the angle needs to be negative
            if index > int(SPLINES / 2):
                angle = -angle
                r = -r

            center = Point(1.0, h)  # center of circle
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
            c for c in cone_msg.cones if abs(c.location.y) < Y_CROP and (c.location.x + LIDAR_DIST) < X_CROP
        ]  # only cones in front of car

        best_spline = 0
        if len(cones) != 0:
            s_error = [0] * len(self.steering_paths)  # error for each spline
            max_error = 1e20

            for n_spline, steering_path in enumerate(self.steering_paths):
                n_spline_points = len(steering_path)
                for cone in cones:
                    # distance from origin
                    distance = sqrt((cone.location.x + LIDAR_DIST) ** 2 + cone.location.y**2)

                    # bin
                    bin = min(n_spline_points - 1, int(round(distance / LINE_LEN * n_spline_points, 0)) - 1)
                    if bin < 0:
                        bin = 0

                    s_error[n_spline] += sqrt(
                        ((cone.location.x + LIDAR_DIST) - steering_path[bin].x) ** 2
                        + (cone.location.y - steering_path[bin].y) ** 2
                    )

                    # draw cones that are being used
                    cv2.drawMarker(
                        self.debug_img,
                        loc_to_img_pt((cone.location.x + LIDAR_DIST), cone.location.y).to_tuple(),
                        (0, 255, 255),
                        markerType=cv2.MARKER_SQUARE,
                        markerSize=5,
                        thickness=5,
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

            if self.last_counter % 1 == 0:
                # ensure that the best spline is next to the last best spline
                if best_spline - self.last_steering_index > 1:
                    best_spline = self.last_steering_index + 1
                elif best_spline - self.last_steering_index < -1:
                    best_spline = self.last_steering_index - 1
                self.last_steering_index = best_spline
                self.last_counter = 0
            self.last_counter += 1

            best_path = self.steering_paths[self.last_steering_index]
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
            steering_angle = STEERING_VALS[self.last_steering_index]

        # publish message
        control_msg = AckermannDriveStamped()
        control_msg.header.stamp = cone_msg.header.stamp
        control_msg.drive.steering_angle = steering_angle
        control_msg.drive.speed = float(speed)
        self.control_publisher.publish(control_msg)

        self.get_logger().debug(f"Total Time: {str(time.perf_counter() - start)}\n")  # log time


def main(args=None):
    rclpy.init(args=args)
    node = PointFitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
