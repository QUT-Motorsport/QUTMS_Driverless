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


class VectorReactiveController(Node):
    Kp_ang: float = 2
    Kp_vel: float = 2
    vel_max: float = 2  # m/s = 7.2km/h
    vel_min: float = vel_max / 2  # m/s
    throttle_max: float = 0.2
    target_cone_count = 3
    r2d: bool = True
    in_dist: float = 1.5 # m
    mid_dist: float = 5 # m

    def __init__(self):
        super().__init__("vector_reactive_controller")

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

        # find points from left side
        midpoints = List[Point]

        if len(left_cones) > 0:
            closest_left = sorted(left_cones, key=lambda c: dist(ORIGIN, cone_to_point(c)))

            if len(closest_left) > 2:                
                for i in range(0, len(closest_left)-1):
                    p1 = cone_to_point(closest_left[i])
                    p2 = cone_to_point(closest_left[i+1])

                    0.5 * (p1+p2)
                    diff = p2-p1
                    R = (diff / (dist(ORIGIN, diff))) * self.in_dist
                    temp = R
                    temp.x = R.y
                    temp.y = -R.x
                    
                    midPoint = (0.5 * (p1+p2)) + temp 
                    midpoints.append(midPoint)

        if len(right_cones) > 0:
            closest_right = sorted(right_cones, key=lambda c: dist(ORIGIN, cone_to_point(c)))

            for i in range(0, len(closest_right)-1):
                p1 = cone_to_point(closest_right[i])
                p2 = cone_to_point(closest_right[i+1])

                0.5 * (p1+p2)
                diff = p2-p1
                R = (diff / (dist(ORIGIN, diff))) * self.in_dist
                temp = R
                temp.x = -R.y
                temp.y = R.x
                
                midPoint = (0.5 * (p1+p2)) + temp 
                midpoints.append(midPoint)

        self.get_logger().debug(f"Midpoints: {midpoints}")

        target: Optional[Point] = None

        # if we found midpoints
        if len(midpoints) > 0
            midpoints = sorted(midpoints, key=lambda c: dist(ORIGIN, c))

            target = ORIGIN
            numMid = 0

            # find all points closer than 5m and average and send it there
            for p in midpoints:
                if dist(ORIGIN, p) < mid_dist:
                    target = target + p
                    numMid = numMid + 1

            # average points
            target = target / numMid

            self.get_logger().debug(f"Target: {target}")


        target: Optional[Point] = None

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
