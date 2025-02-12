import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node

from driverless_msgs.msg import Cone, ConeDetectionStamped
from nav_msgs.msg import Path

from driverless_common.common import QOS_LATEST, angle, midpoint


class PathVisualizer(Node):
    def __init__(self):
        super().__init__("path_visualizer")
        self.get_logger().info("\n--- Path Visualizer Initialised ---")

        # ROS2 Subscribers
        self.blue_path_sub = self.create_subscription(Path, "planning/blue_bounds", self.blue_path_callback, QOS_LATEST)
        self.yellow_path_sub = self.create_subscription(
            Path, "planning/yellow_bounds", self.yellow_path_callback, QOS_LATEST
        )
        self.midline_path_sub = self.create_subscription(
            Path, "planning/midline_path", self.midline_path_callback, QOS_LATEST
        )
        self.cone_detections = self.create_subscription(
            ConeDetectionStamped, "slam/cone_detection", self.cone_detection_callback, QOS_LATEST
        )

        # Path Storage
        self.blue_path = []
        self.yellow_path = []
        self.midline_path = []
        self.blue_cones = []
        self.yellow_cones = []
        self.first_positions = []

        # Received Flags
        self.received_blue_path = False
        self.received_yellow_path = False
        self.received_midline_path = False
        self.received_cones = False

        # Initialize Matplotlib Figure
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        (self.blue_line,) = self.ax.plot([], [], "b-", markersize=4, label="Blue Path")
        (self.yellow_line,) = self.ax.plot([], [], "y-", markersize=4, label="Yellow Path")
        (self.midline_line,) = self.ax.plot([], [], "g-", markersize=4, label="Midline Path")
        (self.blue_cones_plot,) = self.ax.plot([], [], "bo", markersize=4, label="Blue Cones")
        (self.yellow_cones_plot,) = self.ax.plot([], [], "yo", markersize=4, label="Yellow Cones")
        (self.first_positions_plot,) = self.ax.plot([], [], "r-", markersize=4, label="First Positions")

        # Plot settings
        self.ax.set_xlim(-10, 70)
        self.ax.set_ylim(-30, 40)
        self.ax.set_title("Path Visualization")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.legend()
        self.ax.grid(True)

        plt.ion()  # Enable interactive mode
        plt.show()

    # # ✅ Callback Functions
    def blue_path_callback(self, msg):
        self.blue_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.received_blue_path = True
        self.update_plot()
        self.get_logger().info("Received Blue Path!")

    def yellow_path_callback(self, msg):
        self.yellow_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.received_yellow_path = True
        self.update_plot()
        self.get_logger().info("Received Yellow Path!")

    def midline_path_callback(self, msg):
        self.midline_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        if self.midline_path:
            self.first_positions.append(self.midline_path[0])  # Store first position
        self.received_midline_path = True
        self.update_plot()
        self.get_logger().info("Received Midline Path!")

    def cone_detection_callback(self, msg):
        self.blue_cones = [(cone.location.x, cone.location.y) for cone in msg.cones if cone.color == Cone.BLUE]
        self.yellow_cones = [(cone.location.x, cone.location.y) for cone in msg.cones if cone.color == Cone.YELLOW]
        self.received_cones = True
        self.update_plot()
        self.get_logger().info("Received Cones!")

    # ✅ Update Plot Function
    def update_plot(self):
        updated = False

        if self.received_blue_path and self.blue_path:
            x, y = zip(*self.blue_path)
            self.blue_line.set_data(x, y)
            self.received_blue_path = False
            updated = True

        if self.received_yellow_path and self.yellow_path:
            x, y = zip(*self.yellow_path)
            self.yellow_line.set_data(x, y)
            self.received_yellow_path = False
            updated = True

        if self.received_midline_path and self.midline_path:
            x, y = zip(*self.midline_path)
            self.midline_line.set_data(x, y)
            self.received_midline_path = False
            updated = True

        if self.received_cones:
            if self.blue_cones:
                x, y = zip(*self.blue_cones)
                self.blue_cones_plot.set_data(x, y)
            if self.yellow_cones:
                x, y = zip(*self.yellow_cones)
                self.yellow_cones_plot.set_data(x, y)
            if self.first_positions:
                x, y = zip(*self.first_positions)
                self.first_positions_plot.set_data(x, y)
                updated = True
            self.received_cones = False
            updated = True

        if updated:
            self.ax.relim()
            self.ax.autoscale_view()
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.1)  # Ensure real-time updates


def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
