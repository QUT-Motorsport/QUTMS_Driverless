import math
from math import sqrt

import message_filters
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped
from fs_msgs.msg import ControlCommand
from nav_msgs.msg import Odometry


class ControlToSim(Node):
    Kp_vel: float = 2

    def __init__(self):
        super().__init__("control_to_sim")

        # subscribers
        vel_sub = message_filters.Subscriber(self, Odometry, "/testing_only/odom")
        drive_sub = message_filters.Subscriber(self, AckermannDriveStamped, "/driving_command")
        synchronizer = message_filters.ApproximateTimeSynchronizer(
            fs=[drive_sub, vel_sub],
            queue_size=20,
            slop=0.2,
        )
        synchronizer.registerCallback(self.callback)

        # publish to sim
        self.oldpublisher: Publisher = self.create_publisher(ControlCommand, "/control_command", 1)
        self.publisher: Publisher = self.create_publisher(ControlCommand, "/fsds/control_command", 1)

        self.get_logger().info("---Sim control translator initialised---")

    def callback(self, drive_msg: AckermannDriveStamped, vel_msg: Odometry):
        sim_control = ControlCommand()

        # proportional velocity control
        curr_vel = sqrt(vel_msg.twist.twist.linear.x**2 + vel_msg.twist.twist.linear.y**2)
        calc_throttle = self.Kp_vel * (drive_msg.drive.speed - curr_vel)
        if calc_throttle <= 0:
            calc_throttle = 0.0  #  cut throttle

        sim_control.steering = drive_msg.drive.steering_angle / math.pi
        sim_control.throttle = calc_throttle
        sim_control.brake = 0.0
        self.publisher.publish(sim_control)
        self.oldpublisher.publish(sim_control)


def main(args=None):
    rclpy.init(args=args)
    node = ControlToSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
