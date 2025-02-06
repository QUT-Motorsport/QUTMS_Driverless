import numpy as np

import rclpy
from rclpy.node import Node

from driverless_msgs.msg import Cone, ConeDetectionStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class DummyTrackPublisher(Node):
    def __init__(self):
        super().__init__("dummy_track_publisher")
        self.publisher_ = self.create_publisher(ConeDetectionStamped, "slam/cone_detection", 10)
        self.nav_sim_publisher_ = self.create_publisher(Bool, "nav_sim", 10)  # Create publisher for nav_sim
        self.pose_publisher_ = self.create_publisher(PoseStamped, "car/pose", 10)  # Create publisher for car pose
        self.timer = self.create_timer(1, self.publish_dummy_track)  # Publish every second

        # Ellipse parameters
        self.a_blue = 10  # Semi-major axis for blue cones
        self.b_blue = 5  # Semi-minor axis for blue cones
        self.a_yellow = self.a_blue / 3  # Semi-major axis for yellow cones
        self.b_yellow = self.b_blue / 3  # Semi-minor axis for yellow cones
        self.theta = 0  # Initial angle
        self.dtheta = np.pi / 30  # Angle increment

    def publish_dummy_track(self):
        msg = ConeDetectionStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # Define the track points
        blue_cones = [{"x": self.a_blue * np.cos(self.theta), "y": self.b_blue * np.sin(self.theta)}]
        yellow_cones = [
            {"x": self.a_yellow * np.cos(self.theta + np.pi / 2), "y": self.b_yellow * np.sin(self.theta + np.pi / 2)}
        ]

        # Publish nav_sim status
        nav_sim_msg = Bool()
        nav_sim_msg.data = True
        self.nav_sim_publisher_.publish(nav_sim_msg)

        # Add blue cone
        for point in blue_cones:
            cone = Cone()
            cone.location.x = point["x"]
            cone.location.y = point["y"]
            cone.color = Cone.BLUE
            msg.cones.append(cone)
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publishing Blue Cone:\n x:{cone.location.x} y: {cone.location.y}")

        # Add yellow cone
        for point in yellow_cones:
            cone = Cone()
            cone.location.x = point["x"]
            cone.location.y = point["y"]
            cone.color = Cone.YELLOW
            msg.cones.append(cone)
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publishing Yellow Cone:\n x:{cone.location.x} y: {cone.location.y}")

        # Publish car position and orientation
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = self.a_blue * np.cos(self.theta)
        pose_msg.pose.position.y = self.b_blue * np.sin(self.theta)
        pose_msg.pose.orientation.z = np.sin(self.theta / 2)
        pose_msg.pose.orientation.w = np.cos(self.theta / 2)
        self.pose_publisher_.publish(pose_msg)
        self.get_logger().info(f"Publishing Car Location:\n x:{pose_msg.pose.position.x} y: {pose_msg.pose.position.y}")
        self.get_logger().info(
            f"Publishing Car Orentiation:\n z:{pose_msg.pose.orientation.z} w: {pose_msg.pose.orientation.w}"
        )

        # Increment theta for next position
        self.theta += self.dtheta


def main(args=None):
    rclpy.init(args=args)
    node = DummyTrackPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
