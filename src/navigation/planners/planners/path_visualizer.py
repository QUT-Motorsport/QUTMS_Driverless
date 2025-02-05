from queue import Queue
import threading

import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path


class PathVisualizer(Node):
    def __init__(self, plot_queue):
        super().__init__("path_visualizer")
        self.plot_queue = plot_queue
        self.blue_path_sub = self.create_subscription(Path, "planning/blue_bounds", self.blue_path_callback, 10)
        self.yellow_path_sub = self.create_subscription(Path, "planning/yellow_bounds", self.yellow_path_callback, 10)
        self.midline_path_sub = self.create_subscription(Path, "planning/midline_path", self.midline_path_callback, 10)

        self.blue_path = []
        self.yellow_path = []
        self.midline_path = []

        self.received_blue_path = False
        self.received_yellow_path = False
        self.received_midline_path = False

        self.fig, self.ax = plt.subplots()
        (self.blue_line,) = self.ax.plot([], [], "b-", label="Blue Path")
        (self.yellow_line,) = self.ax.plot([], [], "y-", label="Yellow Path")
        (self.midline_line,) = self.ax.plot([], [], "g-", label="Midline Path")

        self.ax.legend()
        self.ax.set_xlim(-20, 20)
        self.ax.set_ylim(-20, 20)
        self.ax.set_title("Path Visualization")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")

        plt.ion()  # Turn on interactive mode
        plt.show()

    def blue_path_callback(self, msg):
        self.blue_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.received_blue_path = True
        self.update_plot()
        self.received_blue_path = False
        self.get_logger().info("Recieved Blue Path!")

    def yellow_path_callback(self, msg):
        self.yellow_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.received_yellow_path = True
        self.update_plot()
        self.received_yellow_path = False
        self.get_logger().info("Recieved Yellow Path!")

    def midline_path_callback(self, msg):
        self.midline_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.received_midline_path = True
        self.update_plot()
        self.received_midline_path = False
        self.get_logger().info("Recieved Midline Path!")

    def update_plot(self):
        if self.received_blue_path:
            x, y = zip(*self.blue_path)
            self.blue_line.set_data(x, y)
        if self.received_yellow_path:
            x, y = zip(*self.yellow_path)
            self.yellow_line.set_data(x, y)
        if self.received_midline_path:
            x, y = zip(*self.midline_path)
            self.midline_line.set_data(x, y)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    plot_queue = Queue()
    node = PathVisualizer(plot_queue)

    def ros_spin():
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    spin_thread = threading.Thread(target=ros_spin)
    spin_thread.start()

    plt.show(block=True)  # Show the plot window and block until it is closed

    spin_thread.join()


if __name__ == "__main__":
    main()
