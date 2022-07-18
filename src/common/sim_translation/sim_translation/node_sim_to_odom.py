import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry


class SimToPose(Node):
    def __init__(self):
        super().__init__("sim_to_pose")

        # subscriber to odom
        self.create_subscription(Odometry, "/testing_only/odom", self.callback, 1)

        # publishers for split pose and velocity
        self.pose_publisher: Publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/zed2i/zed_node/pose_with_covariance", 1
        )
        self.vel_publisher: Publisher = self.create_publisher(TwistWithCovarianceStamped, "/imu/velocity", 1)

        self.get_logger().info("---Sim odometry translator initialised---")

    def callback(self, odom_msg: Odometry):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.pose.pose = odom_msg.pose.pose
        cov: np.ndarray = np.diag(np.random.rand(6) * 0.0001)  # so make some noise
        cov = np.reshape(cov, (6, 6))  # should fill in blanks with 0s??
        pose_msg.pose.covariance = cov.flatten()  # 1x36
        self.pose_publisher.publish(pose_msg)

        vel_msg = TwistWithCovarianceStamped()
        vel_msg.twist.twist = odom_msg.twist.twist
        cov: np.ndarray = np.diag(np.random.rand(6) * 0.0001)
        cov = np.reshape(cov, (6, 6))
        vel_msg.twist.covariance = cov.flatten()
        self.vel_publisher.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimToPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
