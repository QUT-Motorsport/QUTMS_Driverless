from math import atan, pi

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist


class Vel2Ackermann(Node):
    def __init__(self):
        super().__init__("nav_cmd_translator")

        self.create_subscription(Twist, "/control/nav_cmd_vel", self.cmd_callback, 1)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/control/driving_command", 1)

        self.declare_parameter("Kp", 4.0)
        self.declare_parameter("wheelbase", 1.5)
        self.Kp = self.get_parameter("Kp").value
        self.wheelbase = self.get_parameter("wheelbase").value

        self.get_logger().info("---Nav2 control interpreter initalised---")

    def cmd_callback(self, twist_msg: Twist):
        vel = twist_msg.linear.x

        if twist_msg.angular.z == 0 or vel == 0:
            steering = 0.0
        else:
            radius = vel / twist_msg.angular.z
            atan(self.wheelbase / radius) * (180 / pi) * self.Kp  ## MAKE THIS A PARAM

        msg = AckermannDriveStamped()
        # make time for msg id
        # msg.header.stamp =
        msg.header.frame_id = "base_footprint"
        msg.drive.steering_angle = steering
        msg.drive.speed = vel

        self.drive_pub.publish(msg)


def main(args=None):
    # begin ros node
    rclpy.init(args=args)
    node = Vel2Ackermann()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
