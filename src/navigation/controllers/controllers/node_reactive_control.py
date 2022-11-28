import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDrive
from driverless_msgs.msg import Cone, ConeDetectionStamped, Reset

from driverless_common.point import Point, cone_to_point, dist

from typing import List, Optional, Tuple

Colour = Tuple[int, int, int]


ORIGIN = Point(0, 0)

LEFT_CONE_COLOUR = Cone.BLUE
RIGHT_CONE_COLOUR = Cone.YELLOW


class ReactiveController(Node):
    Kp_ang: float = 2
    Kp_vel: float = 2
    vel_max: float = 2  # m/s = 7.2km/h
    vel_min: float = vel_max / 2  # m/s
    throttle_max: float = 0.2
    target_cone_count = 2
    r2d: bool = True

    def __init__(self):
        super().__init__("reactive_controller")

        ebs_test = self.declare_parameter("ebs_control", False).get_parameter_value().bool_value
        self.get_logger().info("EBS Control: " + str(ebs_test))
        if ebs_test:
            self.Kp_ang = -0.1  # shallow steering, straight line
            self.vel_max = 45 / 3.6  # 40km/h in m/s
            self.Kp_vel = 1
            self.vel_min = self.vel_max / 2  # m/s
            self.throttle_max = 0.2

        # self.create_subscription(ConeDetectionStamped, "/vision/cone_detection", self.callback, 1)
        self.create_subscription(ConeDetectionStamped, "/slam/local", self.callback, 1)

        self.reset_sub = self.create_subscription(Reset, "/reset", self.reset_callback, 10)

        self.control_publisher: Publisher = self.create_publisher(AckermannDrive, "/driving_command", 1)

        self.get_logger().info("---Reactive Controller Node Initalised---")
        self.get_logger().info("---Awaing Ready to Drive command *OVERRIDDEN*---")

    def reset_callback(self, msg: Reset):
        self.prev_steering_angle = 0
        self.r2d = True

    def callback(self, cone_msg: ConeDetectionStamped):
        self.get_logger().debug("Received detection")

        # safety critical, set to 0 if not good detection
        control_msg = AckermannDrive()
        control_msg.speed = 0.0
        control_msg.steering_angle = 0.0
        control_msg.acceleration = 0.0
        control_msg.jerk = 1.0

        cones: List[Cone] = cone_msg.cones

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
            steering_angle = self.Kp_ang * np.degrees(np.arctan2(target.y, target.x))
            self.get_logger().debug(f"Target angle: {steering_angle}")

            # publish message
            control_msg.steering_angle = steering_angle
            control_msg.acceleration = self.throttle_max

        self.control_publisher.publish(control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
