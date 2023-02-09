from math import pi, sqrt
import time

import message_filters
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import Reset
from fs_msgs.msg import ControlCommand
from nav_msgs.msg import Odometry

from fs_msgs.srv import Reset as ResetService


class ControlToSim(Node):
    Kp_vel: float = 2
    r2d: bool = False

    def __init__(self):
        super().__init__("control_translator_node")

        # subscribers
        vel_sub = message_filters.Subscriber(self, Odometry, "/fsds/testing_only/odom")
        drive_sub = message_filters.Subscriber(self, AckermannDriveStamped, "/control/driving_command")
        synchronizer = message_filters.ApproximateTimeSynchronizer(
            fs=[drive_sub, vel_sub],
            queue_size=20,
            slop=0.2,
        )
        synchronizer.registerCallback(self.callback)

        self.create_subscription(Reset, "/reset", self.reset_callback, 1)

        # publish to sim
        self.publisher: Publisher = self.create_publisher(ControlCommand, "/fsds/control_command", 1)

        # FSDS reset service
        self.cli = self.create_client(ResetService, "/reset")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Reset srv not available, waiting again...")
        self.req = ResetService.Request()

        self.get_logger().info("---Sim control translator initialised---")

    def reset_callback(self, reset_msg: Reset):
        if self.r2d:
            self.r2d = False
            self.get_logger().info("Ready to drive: FALSE")
        else:
            self.get_logger().info("Ready to drive: TRUE")
            time.sleep(1)
            self.r2d = True

        self.req.wait_on_last_task = False
        self.future = self.cli.call_async(self.req)

    def callback(self, drive_msg: AckermannDriveStamped, vel_msg: Odometry):
        if not self.r2d:
            return

        sim_control = ControlCommand()

        # proportional velocity control
        curr_vel = sqrt(vel_msg.twist.twist.linear.x**2 + vel_msg.twist.twist.linear.y**2)
        calc_throttle = self.Kp_vel * (drive_msg.drive.speed - curr_vel)
        if calc_throttle <= 0:
            calc_throttle = 0.0  #  cut throttle

        sim_control.steering = -0.04 * drive_msg.drive.steering_angle / pi
        sim_control.throttle = calc_throttle
        sim_control.brake = 0.0
        self.publisher.publish(sim_control)


def main(args=None):
    rclpy.init(args=args)
    node = ControlToSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
