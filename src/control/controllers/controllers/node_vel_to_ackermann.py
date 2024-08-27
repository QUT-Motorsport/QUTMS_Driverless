from math import atan, pi

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path


def convert_trans_rot_vel_to_steering_angle(vel, omega, wheelbase):
    if omega == 0 or vel == 0:
        return 0.0

    radius = vel / omega
    return atan(wheelbase / radius) * (180 / pi) * 5


class Vel2Ackermann(Node):
    wheelbase = 1.5  # taken from sim config - measured on car
    initialised = False
    got_plan = False

    def __init__(self):
        super().__init__("nav_cmd_translator")

        self.create_subscription(Twist, "/control/nav_cmd_vel", self.cmd_callback, 1)
        self.create_subscription(Path, "/planning/midline_path", self.planner_callback, 1)
        self.create_timer(0.1, self.timer_callback)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/control/driving_command", 1)

        self.get_logger().info("---Nav2 control interpreter initalised---")

    def planner_callback(self, path):
        self.got_plan = True

    def timer_callback(self):
        if self.initialised:
            return
        if not self.got_plan:
            return

        msg = AckermannDriveStamped()
        # make time for msg id
        # msg.header.stamp =
        msg.header.frame_id = "base_footprint"
        msg.drive.steering_angle = 0.0
        msg.drive.speed = 2.0

        self.drive_pub.publish(msg)

    def cmd_callback(self, twist_msg: Twist):
        self.initialised = True

        vel = twist_msg.linear.x
        steering = convert_trans_rot_vel_to_steering_angle(
            vel,
            twist_msg.angular.z,
            self.wheelbase,
        )

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
