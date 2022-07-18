import math

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDrive

interval = 0.02


class SineController(Node):
    def __init__(self):
        super().__init__("sine_controller")

        self.steering_publisher: Publisher = self.create_publisher(AckermannDrive, "/driving_command", 1)
        self.create_timer(interval, self.timer_cb)
        self.count = 0

        self.get_logger().info("---Sine Controller Node Initalised---")

    def timer_cb(self):
        self.translate = 2 * math.pi / 5
        self.count += interval
        steering_msg = AckermannDrive()
        steering_msg.steering_angle = math.sin(self.count * self.translate) * math.pi
        # self.get_logger().info(f"angle: {steering_msg.steering_angle}")
        self.steering_publisher.publish(steering_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SineController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
