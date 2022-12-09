from math import atan2, cos, sin, sqrt
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import Cone, ConeDetectionStamped, Reset

from driverless_common.node_display import ShutdownNode
from driverless_common.point import Point, cone_to_point, dist

from typing import Any, List, Optional, Tuple

Colour = Tuple[int, int, int]


ORIGIN = Point(0, 0)

LEFT_CONE_COLOUR = Cone.BLUE
RIGHT_CONE_COLOUR = Cone.YELLOW


class BangController(ShutdownNode):
    # config
    target_vel = 30.0 / 3.6
    target_cone_count = 3
    sample_interval = 0.05  # s
    control_action_period = 0.01  # s
    angle_action_threshold = 20  # degrees
    action_command = 40  # degrees

    # operational
    target_angle = 0
    r2d = False

    def __init__(self):
        super().__init__("bang_controller")
        self.create_subscription(ConeDetectionStamped, "/vision/cone_detection2", self.callback, 1)
        self.create_subscription(Reset, "/reset", self.reset_callback, 10)

        self.create_timer(self.sample_interval, self.timer_callback)

        self.control_publisher: Publisher = self.create_publisher(AckermannDriveStamped, "/driving_command", 1)

        self.get_logger().info("---Reactive Controller Node Initalised---")
        self.get_logger().info("---Awaing Ready to Drive command *OVERRIDDEN*---")

    def reset_callback(self, msg: Reset):
        self.prev_steering_angle = 0
        time.sleep(5)
        self.r2d = True

    def detection_callback(self, msg: ConeDetectionStamped):
        cones: List[Cone] = msg.cones

        if msg.header.frame_id == "velodyne":
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
        if len(left_cones) > 0:
            closest_left = sorted(left_cones, key=lambda c: dist(ORIGIN, cone_to_point(c)))[
                min(self.target_cone_count, len(left_cones) - 1)
            ]

        if len(right_cones) > 0:
            closest_right = sorted(right_cones, key=lambda c: dist(ORIGIN, cone_to_point(c)))[
                min(self.target_cone_count, len(right_cones) - 1)
            ]

        # if we have two cones, check if they are greater than 5 meters apart
        # if closest_left is not None and closest_right is not None:
        # if dist(cone_to_point(closest_left), cone_to_point(closest_right)) > 5:
        #     # if so - remove the furthest cone from the targeting
        #     left_dist = dist(ORIGIN, cone_to_point(closest_left))
        #     right_dist = dist(ORIGIN, cone_to_point(closest_right))
        #     if left_dist <= right_dist:
        #         closest_right = None
        #     else:
        #         closest_left = None

        target: Optional[Point] = None
        if closest_left is not None and closest_right is not None:
            target = Point(
                x=closest_left.location.x + (closest_right.location.x - closest_left.location.x) / 2,
                y=closest_left.location.y + (closest_right.location.y - closest_left.location.y) / 2,
            )
        elif closest_left is not None:
            target = Point(
                x=closest_left.location.x,
                y=closest_left.location.y - 3,
            )
        elif closest_right is not None:
            target = Point(
                x=closest_right.location.x,
                y=closest_right.location.y + 3,
            )

        if target is not None:
            # steering control
            self.target_angle = np.degrees(np.arctan2(target.y, target.x))
        else:
            self.target_angle = 0

    def timer_callback(self):
        control_msg = AckermannDriveStamped()
        control_msg.drive.steering_angle = 0
        control_msg.drive.speed = self.target_vel

        if self.target_angle > self.angle_action_threshold:
            control_msg.drive.steering_angle = self.action_command
        elif self.target_angle < -self.angle_action_threshold:
            control_msg.drive.steering_angle = self.action_command

        else:
            pass

        self.control_publisher.publish(control_msg)

        if control_msg.drive.steering_angle != 0:
            time.sleep(self.control_action_period)
            control_msg.drive.steering_angle = 0
            self.control_publisher.publish(control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BangController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
