from math import degrees, pi, sin
import time

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped

from driverless_common.shutdown_node import ShutdownNode


class SineController(Node):
    count = 0
    interval = 0.0025
    pub_interval = 0.01

    def __init__(self):
        super().__init__("sine_controller_node")

        # timed callback
        self.create_timer(self.pub_interval, self.timer_callback)

        self.accel_publisher: Publisher = self.create_publisher(AckermannDriveStamped, "/control/accel_command", 1)
        self.driving_publisher: Publisher = self.create_publisher(AckermannDriveStamped, "/control/driving_command", 1)

        self.get_logger().info("---Sine Controller Node Initalised---")

    def timer_callback(self):
        self.count += self.interval
        control_msg = AckermannDriveStamped()
        control_msg.drive.steering_angle = sin(self.count * pi) * 80  # maximum degrees to turn
        control_msg.drive.acceleration = 0.1
        self.accel_publisher.publish(control_msg)
        self.driving_publisher.publish(control_msg)
        print(self.count)
        print(control_msg.drive.steering_angle)

        if round(self.count, 4) % 4 == 0:
            print("stopped!")


def main(args=None):
    rclpy.init(args=args)
    node = SineController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
