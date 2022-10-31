from math import atan2, cos, pi, sin, sqrt

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDrive
from driverless_msgs.msg import Cone, ConeDetectionStamped

from driverless_common.point import Point

from typing import List, Optional, Tuple

Colour = Tuple[int, int, int]


ORIGIN = Point(0, 0)

LEFT_CONE_COLOUR = Cone.BLUE
RIGHT_CONE_COLOUR = Cone.YELLOW


def dist(a: Point, b: Point) -> float:
    return sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


def cone_to_point(cone: Cone) -> Point:
    return Point(
        cone.location.x,
        cone.location.y,
    )


class PredictiveController(Node):
    Kp_ang: float = -2.5
    Kp_vel: float = 2
    vel_max: float = 2  # m/s = 7.2km/h
    vel_min: float = vel_max / 2  # m/s
    throttle_max: float = 0.1
    prev_steering_angle: float = 0
    Kp_prev: float = 0.4

    def __init__(self):
        super().__init__("predictive_controller")

        ebs_test = self.declare_parameter("ebs_control", False).get_parameter_value().bool_value
        self.get_logger().info("EBS Control: " + str(ebs_test))
        if ebs_test:
            self.Kp_ang = -0.1  # shallow steering, straight line
            self.vel_max = 45 / 3.6  # 40km/h in m/s
            self.Kp_vel = 1
            self.vel_min = self.vel_max / 2  # m/s
            self.throttle_max = 0.2

        # sync subscribers
        self.create_subscription(ConeDetectionStamped, "/vision/cone_detection", self.callback, 1)

        # publishers
        self.control_publisher: Publisher = self.create_publisher(AckermannDrive, "/driving_command", 1)

        self.get_logger().info("---Reactive Controller Node Initalised---")

    def callback(self, cone_msg: ConeDetectionStamped):
        self.get_logger().debug("Received detection")

        # safety critical, set to 0 if not good detection
        control_msg = AckermannDrive()
        control_msg.speed = 0.0
        control_msg.steering_angle = 0.0
        control_msg.acceleration = 0.0
        control_msg.jerk = 1.0  # use as brake

        # weighting for track bounds
        left_weight = 0
        right_weight = 0

        cones: List[Cone] = cone_msg.cones
        left_cones = [c for c in cones if c.color == LEFT_CONE_COLOUR]
        right_cones = [c for c in cones if c.color == RIGHT_CONE_COLOUR]

        # when blue cones is not empty
        if left_cones:
            left_weight = left_cones / len(cones)
        # when yellow cones is not empty
        if left_cones:
            right_weight = -left_cones / len(cones)

        # weight each cone by distance from origin
        left_distances = [dist(ORIGIN, cone_to_point(c)) for c in left_cones]
        right_distances = [dist(ORIGIN, cone_to_point(c)) for c in right_cones]

        # sum weighted distances
        # less weight to further away cones - inverse proportion
        left_weighted_distance = sum([1 / d * left_weight for d in left_distances])
        right_weighted_distance = sum([1 / d * right_weight for d in right_distances])

        closest_left: Optional[Cone] = None
        closest_right: Optional[Cone] = None
        if len(left_cones) > 1:
            # grab second closest
            closest_left = sorted(left_cones, key=lambda c: dist(ORIGIN, cone_to_point(c)))[1]
        elif len(left_cones) > 0:
            closest_left = left_cones[0]

        if len(right_cones) > 1:
            closest_right = sorted(right_cones, key=lambda c: dist(ORIGIN, cone_to_point(c)))[1]
        elif len(right_cones) > 0:
            closest_right = right_cones[0]

        closest_left: Optional[Cone] = None
        closest_right: Optional[Cone] = None
        if len(left_cones) > 1:
            # grab second closest
            closest_left = sorted(left_cones, key=lambda c: dist(ORIGIN, cone_to_point(c)))[1]
        elif len(left_cones) > 0:
            closest_left = left_cones[0]

        if len(right_cones) > 1:
            closest_right = sorted(right_cones, key=lambda c: dist(ORIGIN, cone_to_point(c)))[1]
        elif len(right_cones) > 0:
            closest_right = right_cones[0]

        # if we have two cones, check if they are greater than 5 meters apart
        if closest_left is not None and closest_right is not None:
            if dist(cone_to_point(closest_left), cone_to_point(closest_right)) > 5:
                # if so - remove the furthest cone from the targeting
                left_dist = dist(ORIGIN, cone_to_point(closest_left))
                right_dist = dist(ORIGIN, cone_to_point(closest_right))
                if left_dist <= right_dist:
                    closest_right = None
                else:
                    closest_left = None

        target: Optional[Point] = None
        if closest_left is not None and closest_right is not None:
            target = Point(
                x=closest_left.location.x + (closest_right.location.x - closest_left.location.x) / 2,
                y=closest_left.location.y + (closest_right.location.y - closest_left.location.y) / 2,
            )
        elif closest_left is not None:
            target = Point(
                x=closest_left.location.x,
                y=closest_left.location.y - 2,
            )
        elif closest_right is not None:
            target = Point(
                x=closest_right.location.x,
                y=closest_right.location.y + 2,
            )

        if target is not None:
            # steering control
            steering_angle = self.Kp_ang * ((pi / 2) - atan2(target.x, target.y))
            self.get_logger().debug(f"Target angle: {steering_angle}")

            # # velocity control
            # vel = sqrt(vel_msg.twist.twist.linear.x**2 + vel_msg.twist.twist.linear.y**2)
            # self.get_logger().info(f"True vel: {vel}")

            # # target velocity proportional to angle
            # target_vel: float = self.vel_max - (abs(steering_angle)) * self.Kp_vel
            # if target_vel < self.vel_min:
            #     target_vel = self.vel_min
            # self.get_logger().info(f"Target vel: {target_vel}")

            # # increase proportionally as it approaches target
            # throttle_scalar: float = 1 - (vel / target_vel)
            # if throttle_scalar > 0:
            #     calc_throttle = self.throttle_max * throttle_scalar
            # elif throttle_scalar <= 0:
            #     calc_throttle = 0.0  # if its over maximum, cut throttle

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
