from math import cos, pi, sin, sqrt
import time

import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from sbg_driver.msg import SbgEkfNav
from sensor_msgs.msg import NavSatFix

from driverless_common.common import QOS_ALL

from typing import List, Optional


class VelocityManager:
    state = np.array([0.0, 0.0, 0.0])  # initial pose
    sigma = np.diag([0.0, 0.0, 0.0])
    last_timestamp: Optional[float] = None
    R = np.diag([0.01, 0.01]) ** 2  # motion model
    x: List[float] = []
    y: List[float] = []

    dt: time

    def velocity_callback(self, msg: TwistStamped):
        print(msg)
        if self.last_timestamp is None:
            self.last_timestamp = self.stamp_to_seconds(msg.header.stamp)
            return

        self.dt = self.stamp_to_seconds(msg.header.stamp) - self.last_timestamp

        # rosbag repeat - reset location
        if self.dt < -1:
            self.last_timestamp = self.stamp_to_seconds(msg.header.stamp)
            self.state[0:3] = [0.0, 0.0, 0.0]
            return

        self.last_timestamp = self.stamp_to_seconds(msg.header.stamp)

        # predict with velocity
        self.predict(msg)

    def predict(self, vel_msg: TwistStamped):
        """
        Predict step of the EKF
        * param vel_msg: WSSVelocity message containing avg wheel speeds
        """

        ddist = sqrt(vel_msg.twist.linear.x**2 + vel_msg.twist.linear.y**2) * self.dt  # distance
        dtheta = vel_msg.twist.angular.z * self.dt  # change in angle

        self.state[0] = self.state[0] + ddist * cos(self.state[2])
        self.state[1] = self.state[1] + ddist * sin(self.state[2])
        self.state[2] = self.wrap_to_pi(self.state[2] + dtheta)

        self.x.append(self.state[0])
        self.y.append(self.state[1])

    @staticmethod
    def stamp_to_seconds(stamp) -> float:
        return stamp.sec + stamp.nanosec / 1e9

    @staticmethod
    def wrap_to_pi(angle: float) -> float:  # in rads
        return (angle + pi) % (2 * pi) - pi


class NavSatFixManager:
    x: List[float] = []
    y: List[float] = []

    def nav_sat_fix_callback(self, msg: NavSatFix):
        self.x.append(msg.latitude)
        self.y.append(msg.longitude)


class EkfNavManager:
    x: List[float] = []
    y: List[float] = []

    def ekf_nav_callback(self, msg: SbgEkfNav):
        self.x.append(msg.latitude)
        self.y.append(msg.longitude)


class SbgPerformance(Node):
    vel_manager: VelocityManager
    nav_sat_fix_manager: NavSatFixManager
    ekf_nav_manager: EkfNavManager

    fig = plt.figure()
    axes = fig.subplots(3)

    def __init__(self) -> None:
        super().__init__("sbg_performance")

        self.vel_manager = VelocityManager()
        self.create_subscription(TwistStamped, "imu/velocity", self.vel_manager.velocity_callback, QOS_ALL)

        self.ekf_nav_manager = EkfNavManager()
        self.create_subscription(SbgEkfNav, "sbg/ekf_nav", self.ekf_nav_manager.ekf_nav_callback, QOS_ALL)

        self.nav_sat_fix_manager = NavSatFixManager()
        self.create_subscription(NavSatFix, "imu/nav_sat_fix", self.nav_sat_fix_manager.nav_sat_fix_callback, QOS_ALL)

        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.update_graph)

    def update_graph(self):
        self.axes[0].clear()
        self.axes[0].plot(self.vel_manager.x, self.vel_manager.y)

        self.axes[1].clear()
        self.axes[1].plot(self.nav_sat_fix_manager.x, self.nav_sat_fix_manager.y)

        self.axes[2].clear()
        self.axes[2].plot(self.ekf_nav_manager.x, self.ekf_nav_manager.y)

        plt.show(block=False)
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = SbgPerformance()
    rclpy.spin(node)
    plt.close(node.fig)
    node.destroy_node()
    rclpy.shutdown()
