from math import atan, cos, pi, sin, sqrt
import time

from colour import Color
import cv2
import numpy as np
from scipy.interpolate import UnivariateSpline
from scipy.spatial import distance
from sklearn.linear_model import LinearRegression

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import Cone, ConeDetectionStamped, Reset
from sensor_msgs.msg import Image

from driverless_common.draw import draw_markers, draw_steering, loc_to_img_pt
from driverless_common.point import Point, cone_to_point, dist
from driverless_common.shutdown_node import ShutdownNode

from typing import Any, List, Optional, Tuple

cv_bridge = CvBridge()  # translate ROS image messages to OpenCV

red = Color("red")
blue = Color("blue")
col_range = list(blue.range_to(red, 100))

ORIGIN = Point(0, 0)

LEFT_CONE_COLOUR = Cone.BLUE
RIGHT_CONE_COLOUR = Cone.YELLOW

left_mid_col = (245, 215, 66)
right_mid_col = (66, 194, 245)

LOOKAHEAD = 6  # m


class BangController(Node):
    # config
    target_vel = 10.0 / 3.6
    target_cone_count = 3
    sample_interval = 0.5  # s
    control_action_period = 0.25  # s
    angle_action_threshold = 0.0  # degrees
    action_command = 20.0  # degrees

    # Kp_ang: float = -1.0
    # Kp_vel: float = 2.0
    # vel_max: float = 30 / 3.6  # kmph
    # vel_min: float = vel_max / 2  # m/s
    # throttle_max: float = 0.2
    # target_cone_count = 3
    # r2d: bool = True
    in_dist: float = 1.5  # m
    # mid_dist: float = 5  # m
    # prev_angle: float = 0.0

    # operational
    target_angle = 0
    r2d = False

    def __init__(self):
        super().__init__("bang_controller_node")
        # self.create_subscription(ConeDetectionStamped, "/vision/cone_detection", self.detection_callback, 1)
        self.create_subscription(ConeDetectionStamped, "/lidar/cone_detection", self.detection_callback, 1)
        self.create_subscription(Reset, "/system/reset", self.reset_callback, 10)
        self.vector_publisher: Publisher = self.create_publisher(Image, "/debug_imgs/vector_reactive_img", 1)

        self.create_timer(self.sample_interval, self.timer_callback)

        self.control_publisher: Publisher = self.create_publisher(AckermannDriveStamped, "/control/driving_command", 1)

        self.get_logger().info("---Reactive Controller Node Initalised---")
        self.get_logger().info("---Awaing Ready to Drive command *OVERRIDDEN*---")

    def reset_callback(self, msg: Reset):
        self.prev_steering_angle = 0
        time.sleep(5)
        self.r2d = True

    def detection_callback(self, msg: ConeDetectionStamped):
        cones: List[Cone] = msg.cones

        cones = [c for c in cones if c.location.x < 10]

        if msg.header.frame_id == "velodyne":
            # change colour of cones based on on y pos as this is lidar scan
            for i in range(len(cones)):
                if cones[i].location.y > 0:
                    cones[i].color = LEFT_CONE_COLOUR
                else:
                    cones[i].color = RIGHT_CONE_COLOUR

        left_cones = [c for c in cones if c.color == LEFT_CONE_COLOUR]
        right_cones = [c for c in cones if c.color == RIGHT_CONE_COLOUR]
        orange_cones = [c for c in cones if c.color == Cone.ORANGE_BIG]

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

                diff = p2 - p1
                mult = 1.0 / dist(ORIGIN, diff)
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

                diff = p2 - p1
                mult = 1.0 / dist(ORIGIN, diff)
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

        target: Optional[Point] = None

        midpoints.append(ORIGIN)
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
            x = np.array([p.x for p in midpoints]).reshape((-1, 1))
            y = np.array([p.y for p in midpoints])

            model = LinearRegression().fit(x, y)
            grad = model.coef_[0]

            self.target_angle = np.degrees(np.arctan(grad))
            debug_img = draw_steering(debug_img, self.target_angle, 0)  # draw steering angle and vel data on image

        self.vector_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))
        self.get_logger().info(f"Target: {self.target_angle}")

    def timer_callback(self):
        control_msg = AckermannDriveStamped()
        control_msg.drive.steering_angle = 0.0
        control_msg.drive.speed = self.target_vel

        if self.target_angle > self.angle_action_threshold:
            control_msg.drive.steering_angle = self.action_command
        elif self.target_angle < -self.angle_action_threshold:
            control_msg.drive.steering_angle = -self.action_command
        else:
            pass

        self.control_publisher.publish(control_msg)
        self.get_logger().info(f"First Send: {control_msg.drive.steering_angle}")

        if control_msg.drive.steering_angle != 0:
            time.sleep(self.control_action_period)
            control_msg.drive.steering_angle = 0.0
            self.control_publisher.publish(control_msg)
            self.get_logger().info(f"Second Send: {control_msg.drive.steering_angle}")


def main(args=None):
    rclpy.init(args=args)
    node = BangController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
