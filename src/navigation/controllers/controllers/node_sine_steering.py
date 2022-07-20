import math

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDrive


class SineController(Node):
    count = 0
    interval = 0.02

    def __init__(self):
        super().__init__("sine_controller")

        # timed callback
        self.create_timer(self.interval, self.timer_cb)

        self.steering_publisher: Publisher = self.create_publisher(AckermannDrive, "/driving_command", 1)

        self.get_logger().info("---Sine Controller Node Initalised---")

    def timer_callback(self):
        self.translate = 2 * math.pi / 5
        self.count += self.interval
        steering_msg = AckermannDrive()
        steering_msg.steering_angle = math.sin(self.count * self.translate) * math.pi
        self.get_logger().info("Angle: " + str(steering_msg.steering_angle))
        self.steering_publisher.publish(steering_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SineController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
