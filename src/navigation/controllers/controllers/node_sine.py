import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.timer import Timer

import math

from ackermann_msgs.msg import AckermannDrive


class SineControllerNode(Node):
    def __init__(self):
        super().__init__("SineController")

        self.steering_publisher: Publisher = self.create_publisher(
            AckermannDrive, "steering", 1)
        self.get_logger().info("Sine Controller")
        self.create_timer(0.01, self.timer_cb)
        self.count = 0

    def timer_cb(self):
        self.translate = 2 * math.pi / 5
        self.count += 0.01
        steering_msg = AckermannDrive()
        steering_msg.steering_angle = math.sin(
            self.count * self.translate) * math.pi
        # self.get_logger().info(f"angle: {steering_msg.steering_angle}")
        self.steering_publisher.publish(steering_msg)


def main(args=None):
    rclpy.init(args=args)

    sine_controller_node = SineControllerNode()

    rclpy.spin(sine_controller_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
