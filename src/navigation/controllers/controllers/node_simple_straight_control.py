import numpy as np

import rclpy
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import Cone, ConeDetectionStamped

from driverless_common.point import Point
from driverless_common.shutdown_node import ShutdownNode

from typing import List, Tuple

Colour = Tuple[int, int, int]


ORIGIN = Point(0, 0)

LEFT_CONE_COLOUR = Cone.BLUE
RIGHT_CONE_COLOUR = Cone.YELLOW


class SimpleStraightController(ShutdownNode):
    Kp_ang: float = 2.0
    target_accel: float = 0.5
    x_roi = 3  # m +- horizontal (left right)
    y_roi = 6  # m +  forward

    def __init__(self):
        super().__init__("simple_straight_controller")
        # self.create_subscription(ConeDetectionStamped, "vision/cone_detection2", self.callback, 1)
        self.create_subscription(ConeDetectionStamped, "lidar/cone_detection", self.callback, 1)
        # self.create_subscription(ConeDetectionStamped, "slam/local", self.callback, 1)

        self.control_publisher: Publisher = self.create_publisher(AckermannDriveStamped, "driving_command", 1)
        self.accel_publisher: Publisher = self.create_publisher(AckermannDriveStamped, "accel_command", 1)

        self.get_logger().info("---Simple Straight Controller Node Initalised---")

    def callback(self, msg: ConeDetectionStamped):
        self.get_logger().debug("Received detection")
        steering_angle = 0.0

        cones: List[Cone] = [cone for cone in msg.cones if abs(cone.x) <= self.x_roi and cone.y <= self.y_roi]

        if msg.header.frame_id == "velodyne":
            # change colour of cones based on on y pos as this is lidar scan
            for i in range(len(cones)):
                if cones[i].location.y > 0:
                    cones[i].color = LEFT_CONE_COLOUR
                else:
                    cones[i].color = RIGHT_CONE_COLOUR

        left_cones = [c for c in cones if c.color == LEFT_CONE_COLOUR]
        right_cones = [c for c in cones if c.color == RIGHT_CONE_COLOUR]

        if len(left_cones) > 0 and len(right_cones) > 0:
            left_x = np.mean([cone.location.x for cone in left_cones])
            right_x = np.mean([cone.location.x for cone in right_cones])

            center = (left_x + right_x) / 2.0

            steering_angle = self.Kp_ang * center

        # publish message
        control_msg = AckermannDriveStamped()
        control_msg.header.stamp = msg.header.stamp
        control_msg.drive.steering_angle = steering_angle
        control_msg.drive.speed = 0.0
        control_msg.drive.acceleration = self.target_accel

        self.control_publisher.publish(control_msg)
        self.accel_publisher.publish(control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleStraightController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
