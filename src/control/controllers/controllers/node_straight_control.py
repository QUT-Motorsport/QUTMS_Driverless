from math import atan2, cos, sin, sqrt
import time

import cv2
import numpy as np
from sklearn.linear_model import LinearRegression

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import Cone, ConeDetectionStamped, Reset

from driverless_common.draw import draw_markers, loc_to_img_pt
from driverless_common.point import Point, cone_to_point, dist
from driverless_common.shutdown_node import ShutdownNode

from typing import List, Optional, Tuple

Colour = Tuple[int, int, int]

left_mid_col = (245, 215, 66)
right_mid_col = (66, 194, 245)

ORIGIN = Point(0, 0)

LEFT_CONE_COLOUR = Cone.BLUE
RIGHT_CONE_COLOUR = Cone.YELLOW


class StraightControl(Node):
    Kp_ang: float = 2.0
    vel_max: float = 3.0  # m/s = 7.2km/h
    vel_min: float = vel_max / 2  # m/s
    throttle_max: float = 0.2
    target_cone_count = 3
    r2d: bool = False

    def __init__(self):
        super().__init__("straight_control_node")

        self.ebs_test = self.declare_parameter("ebs_control", False).get_parameter_value().bool_value
        self.get_logger().info("EBS Control: " + str(self.ebs_test))

        if self.ebs_test:
            self.Kp_ang = 10.0
            self.vel_max = 30.0 / 3.6
            self.create_subscription(ConeDetectionStamped, "/vision/cone_detection2", self.callback, 1)
            self.target_cone_count = 2
        else:
            self.create_subscription(ConeDetectionStamped, "/slam/local_map", self.callback, 1)

        self.reset_sub = self.create_subscription(Reset, "/system/reset", self.reset_callback, 10)

        self.control_publisher: Publisher = self.create_publisher(AckermannDriveStamped, "/control/driving_command", 1)

        self.get_logger().info("---Reactive Controller Node Initalised---")
        self.get_logger().info("---Awaing Ready to Drive command *OVERRIDDEN*---")

    def reset_callback(self, msg: Reset):
        self.prev_steering_angle = 0
        time.sleep(5)
        self.r2d = True

    def callback(self, msg: ConeDetectionStamped):
        self.get_logger().debug("Received detection")

        # safety critical, set to 0 if not good detection
        speed = 0.0
        steering_angle = 0.0

        cones: List[Cone] = msg.cones

        if msg.header.frame_id == "velodyne" and self.ebs_test:
            # change colour of cones based on on y pos as this is lidar scan
            for i in range(len(cones)):
                if cones[i].location.y > 0:
                    cones[i].color = LEFT_CONE_COLOUR
                else:
                    cones[i].color = RIGHT_CONE_COLOUR

        left_cones = [c for c in cones if c.color == LEFT_CONE_COLOUR]
        right_cones = [c for c in cones if c.color == RIGHT_CONE_COLOUR]

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

        # midpoints.append(ORIGIN)

        grad = 0

        if len(midpoints) > 1:
            # more than 1 midpoint visible, fit linear regression to them
            x = []
            y = []
            for p in midpoints:
                x.append(p.x)
                y.append(p.y)

            y = np.array(y)
            x = np.array(x).reshape((-1, 1))

            model = LinearRegression().fit(x, y)
            r_sq = model.score(x, y)
            c = model.intercept_
            grad = model.coef_[0]

            self.get_logger().debug(f"Best fit: m: {grad} c:{c}")

            cv2.line(
                debug_img,
                loc_to_img_pt(0, c).to_tuple(),
                loc_to_img_pt(-c / grad, 0).to_tuple(),
                (0, 255, 255),
                thickness=2,
            )

        if (grad != 0) and self.r2d:
            # steering control
            steering_angle = self.Kp_ang * np.degrees(np.arctan(grad))
            self.get_logger().debug(f"Target angle: {steering_angle}")
            speed = self.vel_max

        # publish message
        control_msg = AckermannDriveStamped()
        control_msg.header.stamp = msg.header.stamp
        control_msg.drive.steering_angle = steering_angle
        control_msg.drive.speed = speed

        self.control_publisher.publish(control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StraightControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
