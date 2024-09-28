from math import degrees, pi, sin

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped

class SineController(Node):
    count = 0
    interval = 0.02
    pub_interval = 0.05

    def __init__(self):
        super().__init__("sine_controller_node")

        # timed callback
        self.create_timer(self.pub_interval, self.timer_callback)

        self.steering_publisher = self.create_publisher(AckermannDriveStamped, "/control/driving_command", 1)
        self.driving_publisher = self.create_publisher(AckermannDriveStamped, "/control/accel_command", 1)

        self.get_logger().info("---Sine controller node initalised---")

    def timer_callback(self):
        self.count += self.interval
        control_msg = AckermannDriveStamped()
        control_msg.drive.steering_angle = sin(self.count * pi) * -60  # maximum degrees to turn
        control_msg.drive.acceleration = 0.1
        self.driving_publisher.publish(control_msg)
        self.steering_publisher.publish(control_msg)
        self.get_logger().info(f"Published angle {control_msg.drive.steering_angle}")


def main(args=None):
    rclpy.init(args=args)
    node = SineController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
