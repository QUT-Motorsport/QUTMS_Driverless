from math import pi, sin

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped

from driverless_common.shutdown_node import ShutdownNode


class SineController(ShutdownNode):
    count = 0
    interval = 0.02

    def __init__(self):
        super().__init__("sine_controller")

        # timed callback
        self.create_timer(self.interval, self.timer_callback)

        self.sine_publisher: Publisher = self.create_publisher(AckermannDriveStamped, "/driving_command", 1)

        self.get_logger().info("---Sine Controller Node Initalised---")

    def timer_callback(self):
        translate = 2 * pi / 5
        self.count += self.interval
        control_msg = AckermannDriveStamped()
        control_msg.drive.steering_angle = sin(self.count * translate) * pi
        control_msg.drive.speed = 0.5
        self.get_logger().info(
            "Angle: " + str(control_msg.drive.steering_angle) + "\tSpeed: " + str(control_msg.drive.speed)
        )
        self.sine_publisher.publish(control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SineController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
