import random

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped

from driverless_common.shutdown_node import ShutdownNode


class RandomController(Node):
    target: float = 0.0

    change_interval = 1  # s
    pub_interval = 0.1  # s

    def __init__(self):
        super().__init__("random_controller_node")

        # timed callback
        self.create_timer(self.change_interval, self.change_callback)
        self.create_timer(self.pub_interval, self.pub_callback)

        self.drive_publisher: Publisher = self.create_publisher(AckermannDriveStamped, "/control/driving_command", 1)

        self.get_logger().info("---Random Controller Node Initalised---")

    def pub_callback(self):
        control_msg = AckermannDriveStamped()
        control_msg.drive.steering_angle = self.target
        self.drive_publisher.publish(control_msg)

    def change_callback(self):
        self.target = float(random.randrange(-80, 80))


def main(args=None):
    rclpy.init(args=args)
    node = RandomController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
