import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path


class PathVisualizer(Node):
    def __init__(self):
        super().__init__("path_visualizer")

        # ROS2 Subscribers
        self.blue_path_sub = self.create_subscription(Path, "planning/blue_bounds", self.blue_path_callback, 10)
        self.yellow_path_sub = self.create_subscription(Path, "planning/yellow_bounds", self.yellow_path_callback, 10)
        self.midline_path_sub = self.create_subscription(Path, "planning/midline_path", self.midline_path_callback, 10)

        # Path Storage
        self.blue_path = []
        self.yellow_path = []
        self.midline_path = []

        # Received Flags
        self.received_blue_path = False
        self.received_yellow_path = False
        self.received_midline_path = False

        # Initialize Matplotlib Figure
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        (self.blue_line,) = self.ax.plot([], [], "b-", label="Blue Path")
        (self.yellow_line,) = self.ax.plot([], [], "y-", label="Yellow Path")
        (self.midline_line,) = self.ax.plot([], [], "g-", label="Midline Path")

        # Plot settings
        self.ax.set_xlim(-20, 80)
        self.ax.set_ylim(-60, 60)
        self.ax.set_title("Path Visualization")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.legend()
        self.ax.grid(True)

        plt.ion()  # Enable interactive mode
        plt.show()

    # ✅ Callback Functions
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
        self.received_midline_path = True
        self.update_plot()
        self.get_logger().info("Received Midline Path!")

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
