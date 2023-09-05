from math import atan2, cos, hypot, pi, sin, sqrt
import time

from geodesy.utm import UTMPoint, fromLatLong
import numpy as np
from transforms3d.euler import euler2quat, quat2euler

import message_filters
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from geometry_msgs.msg import Point, PointStamped, PoseWithCovarianceStamped, Quaternion, TransformStamped, TwistStamped
from sbg_driver.msg import SbgEkfEuler, SbgEkfNav, SbgGpsPos, SbgGpsVel, SbgImuData, SbgMag
from sensor_msgs.msg import Imu, NavSatFix


class SBGDebug(Node):
    def __init__(self):
        super().__init__("sbg_angle_node")

        # sync subscribers
        self.create_subscription(SbgEkfEuler, "/sbg/ekf_euler", self.ekf_euler_callback, 10)

        # publishers
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/pose/euler_to_pose", 10)

    def ekf_euler_callback(self, msg):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = msg.header
        # convert euler angles to quaternion
        q = euler2quat(msg.angle.x, msg.angle.y, msg.angle.z)
        pose_msg.pose.pose.orientation.w = q[0]
        pose_msg.pose.pose.orientation.x = q[1]
        pose_msg.pose.pose.orientation.y = q[2]
        pose_msg.pose.pose.orientation.z = q[3]

        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SBGDebug()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
