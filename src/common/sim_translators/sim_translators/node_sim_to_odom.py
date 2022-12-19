from math import pi

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from driverless_msgs.msg import MotorRPM, WSSVelocity
from fs_msgs.msg import WheelStates
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry

WHEEL_DIAMETER = 0.4064


class SimToOdom(Node):
    def __init__(self):
        super().__init__("sim_to_odom")

        # subscriber to odom
        self.create_subscription(Odometry, "/fsds/testing_only/odom", self.odom_callback, 1)
        self.create_subscription(WheelStates, "/fsds/wheel_states", self.wheel_callback, 1)

        # publishers for split pose and velocity
        self.pose_publisher: Publisher = self.create_publisher(
            PoseWithCovarianceStamped, "zed2i/zed_node/pose_with_covariance", 10
        )
        self.vel_publisher: Publisher = self.create_publisher(TwistStamped, "/imu/velocity", 10)
        self.rpm_publisher: Publisher = self.create_publisher(MotorRPM, "motor_rpm", 10)
        self.wss_vel_publisher: Publisher = self.create_publisher(WSSVelocity, "vehicle_wss", 10)

        self.get_logger().info("---Sim odometry translator initialised---")

    def odom_callback(self, odom_msg: Odometry):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = odom_msg.header
        pose_msg.header.frame_id = "base_link"
        pose_msg.pose.pose = odom_msg.pose.pose
        cov: np.ndarray = np.diag(np.random.rand(6) * 0.1)  # so make some noise
        cov = np.reshape(cov, (6, 6))  # should fill in blanks with 0s??
        pose_msg.pose.covariance = cov.flatten()  # 1x36
        self.pose_publisher.publish(pose_msg)

        vel_msg = TwistStamped()
        vel_msg.header = odom_msg.header
        vel_msg.header.frame_id = "imu_link_ned"
        vel_msg.twist = odom_msg.twist.twist
        vel_msg.twist.angular.z = -odom_msg.twist.twist.angular.z
        self.vel_publisher.publish(vel_msg)

    def wheel_callback(self, wheel_msg: WheelStates):
        # indiv motor rpm
        fl_rpm = int(wheel_msg.fl_rpm * (21.0 * 4.50))
        fl_rpm_msg = MotorRPM(index=0, rpm=fl_rpm)
        fr_rpm = int(wheel_msg.fr_rpm * (21.0 * 4.50))
        fr_rpm_msg = MotorRPM(index=1, rpm=fr_rpm)
        rl_rpm = int(wheel_msg.rl_rpm * (21.0 * 4.50))
        rl_rpm_msg = MotorRPM(index=2, rpm=rl_rpm)
        rr_rpm = int(wheel_msg.rr_rpm * (21.0 * 4.50))
        rr_rpm_msg = MotorRPM(index=3, rpm=rr_rpm)
        self.rpm_publisher.publish(fl_rpm_msg)
        self.rpm_publisher.publish(fr_rpm_msg)
        self.rpm_publisher.publish(rl_rpm_msg)
        self.rpm_publisher.publish(rr_rpm_msg)

        # wheel speed avg velocity
        wss_vel = WSSVelocity()
        wss_vel.header.stamp = wheel_msg.header.stamp
        vel = (
            wheel_msg.fr_rpm * pi * WHEEL_DIAMETER / 60
            + wheel_msg.fl_rpm * pi * WHEEL_DIAMETER / 60
            + wheel_msg.rl_rpm * pi * WHEEL_DIAMETER / 60
            + wheel_msg.rr_rpm * pi * WHEEL_DIAMETER / 60
        )
        wss_vel.velocity = vel / 4
        if wss_vel.velocity < 0.01:
            wss_vel.velocity = 0.0
        self.wss_vel_publisher.publish(wss_vel)


def main(args=None):
    rclpy.init(args=args)
    node = SimToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
