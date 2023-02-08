import math
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped

from driverless_common.shutdown_node import ShutdownNode


class StepController(Node):
    target: float = -7500.0

    change_interval = 5  # s
    pub_interval = 0.01  # s

    step = 0
    max_ang = 80.0
    inc = 750

    def __init__(self):
        super().__init__("step_controller_node")

        # timed callback
        self.create_timer(self.change_interval, self.change_callback)
        self.create_timer(self.pub_interval, self.pub_callback)

        self.drive_publisher: Publisher = self.create_publisher(AckermannDriveStamped, "/driving_command", 1)

        self.get_logger().info("---Step Controller Node Initalised---")

    def pub_callback(self):
        control_msg = AckermannDriveStamped()
        control_msg.drive.steering_angle = self.target
        self.drive_publisher.publish(control_msg)

    def change_callback(self):
        if self.step == 0:
            self.target = 0.0
            self.step = 1
        elif self.step == 1:
            self.target = self.max_ang
            self.step = 2
        elif self.step == 2:
            self.target = -self.max_ang
            self.step = 0

        # if self.target < 7500:
        #     self.target += self.inc
        #     print(self.target)


def main(args=None):
    rclpy.init(args=args)
    node = StepController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
