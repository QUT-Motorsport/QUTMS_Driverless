import math

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDrive
from fs_msgs.msg import ControlCommand


class ControlToSim(Node):
    def __init__(self):
        super().__init__("control_to_sim")

        # sub to drive
        self.create_subscription(AckermannDrive, "/driving_command", self.control_callback, 1)

        # publish to sim
        self.publisher: Publisher = self.create_publisher(ControlCommand, "/control_command", 1)

        self.get_logger().info("---Sim control translator initialised---")

    def control_callback(self, msg: AckermannDrive):
        sim_control = ControlCommand()
        sim_control.steering = msg.steering_angle / math.pi
        sim_control.throttle = msg.acceleration  # accel used for throttle
        sim_control.brake = msg.jerk  # jerk used for brake
        self.publisher.publish(sim_control)


def main(args=None):
    rclpy.init(args=args)
    node = ControlToSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
