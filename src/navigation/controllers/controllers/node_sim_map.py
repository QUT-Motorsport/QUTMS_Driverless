import math

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from fs_msgs.msg import ControlCommand
from ackermann_msgs.msg import AckermannDrive


class SimMapControllerNode(Node):
    def __init__(self):
        super().__init__("SineController")

        self.create_subscription(ControlCommand, "control_command", self.control_callback, 1)
        self.steering_publisher: Publisher = self.create_publisher(AckermannDrive, "steering", 1)
        self.get_logger().info("Sim Map Controller")

    def control_callback(self, msg: ControlCommand):
        steering_msg = AckermannDrive()
        steering_msg.steering_angle = msg.steering*math.pi*3
        self.steering_publisher.publish(steering_msg)


def main(args=None):
    rclpy.init(args=args)

    sim_map_controller_node = SimMapControllerNode()

    rclpy.spin(sim_map_controller_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
