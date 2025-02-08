import matplotlib.pyplot as plt
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
        self.nav_sim_publisher_ = self.create_publisher(Bool, "nav_sim", 10)
        self.pose_publisher_ = self.create_publisher(PoseStamped, "car/pose", 10)
        self.timer = self.create_timer(1, self.publish_dummy_track)  # Publish every second

        # Publish nav_sim status (tell node_ft_planner, we're simulating)
        nav_sim_msg = Bool()
        nav_sim_msg.data = True
        self.nav_sim_publisher_.publish(nav_sim_msg)

        # Plotting flag
        self.plotting_enabled = True  # Set this to False to disable plotting

        # Define track list of cones and car_positions
        self.yellow_cones = [
            (0, 0),
            (5, 3),
            (10, 8),
            (15, 15),
            (20, 22),
            (25, 28),
            (30, 32),
            (35, 35),
            (40, 36),
            (45, 35),
            (50, 32),
            (55, 28),
            (60, 22),
            (63, 15),
            (65, 8),
            (66, 2),
            (65, -5),
            (62, -12),
            (58, -18),
            (53, -23),
            (47, -27),
            (40, -30),
            (33, -31),
            (26, -30),
            (19, -27),
            (13, -22),
            (8, -16),
            (4, -9),
            (1, -3),
            (0, 0),
        ]  # Closing the loop

        self.blue_cones = [
            (0, -2),
            (5, 1),
            (10, 6),
            (15, 12),
            (20, 18),
            (25, 24),
            (30, 28),
            (35, 31),
            (40, 32),
            (45, 31),
            (50, 28),
            (55, 24),
            (60, 18),
            (63, 12),
            (65, 6),
            (66, 0),
            (65, -6),
            (62, -10),
            (58, -15),
            (53, -20),
            (47, -24),
            (40, -27),
            (33, -28),
            (26, -27),
            (19, -24),
            (13, -20),
            (8, -14),
            (4, -7),
            (1, -2),
            (0, -2),
        ]  # Closing the loop

        # Calculate car_position from the list of cones
        self.car_positions = []
        for i in range(len(self.yellow_cones)):
            x_mid = (self.yellow_cones[i][0] + self.blue_cones[i][0]) / 2
            y_mid = (self.yellow_cones[i][1] + self.blue_cones[i][1]) / 2
            self.car_positions.append((x_mid, y_mid))

        # Calculate the cars_orientation as quaternions (convert to euler in node_ft_planner)
        self.car_orientation = []

        self.car_orientation = []
        for i in range(len(self.yellow_cones) - 1):
            dx = self.car_positions[i + 1][0] - self.car_positions[i][0]
            dy = self.car_positions[i + 1][1] - self.car_positions[i][1]
            theta = np.arctan2(dy, dx)

            # Convert yaw to quaternion (only rotation about the Z-axis)
            w = np.cos(theta / 2)
            x = 0
            y = 0
            z = np.sin(theta / 2)

            self.car_orientation.append({"theta": theta, "w": w, "x": x, "y": y, "z": z})

        # Initialize plot (if plotting is enabled)
        if self.plotting_enabled:
            self.fig, self.ax = plt.subplots(figsize=(10, 6))

            self.ax.set_xlim(-20, 80)
            self.ax.set_ylim(-60, 60)
            self.ax.set_title("Cone and Car Position Visualization")
            self.ax.set_xlabel("X")
            self.ax.set_ylabel("Y")

            self.ax.legend(["Blue Cones", "Yellow Cones", "Car Position"])
            self.ax.grid(True)

            plt.ion()  # Turn on interactive mode
            plt.show()

        # Initialize index for incremental publishing
        self.current_index = 0

    def publish_dummy_track(
        self,
    ):  # Timer/Main Callback, publishes dummy track at rate specified by timer subscrober (Hz)
        msg = ConeDetectionStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # cone_msg : function to publish cone locations as ConeDetectionStamped msg
        def cone_msg(cone_x, cone_y, cone_color):
            cone = Cone()
            cone.location.x = float(cone_x)
            cone.location.y = float(cone_y)
            cone.location.z = 0.0  # Ensure z is set to 0.0
            cone.color = cone_color
            return cone

        # pose_msg : function to publish pose locations (car_orientation & car_position) as PoseStamped msg
        def pose_msg(car_x, car_y, car_orien_w, car_orien_x, car_orien_y, car_orien_z):
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position.x = car_x
            pose_msg.pose.position.y = car_y
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation.w = car_orien_w
            pose_msg.pose.orientation.x = car_orien_x
            pose_msg.pose.orientation.y = car_orien_y
            pose_msg.pose.orientation.z = car_orien_z
            self.pose_publisher_.publish(pose_msg)
            self.get_logger().info(
                f"Publishing Car Position: x={self.car_positions[i][0]}, y={self.car_positions[i][1]}"
            )

        ##### PUBLISHING ####
        # Publish cones from cone list, x cones ahead of current_index (using publishing functions)
        # NOTE: In any of the planning scripts, cone locations are not retained, nor does the path planner retain the cone locations.
        # NOTE: The car path plans based on the current list of cones and car pose.
        n_publish_ahead = 3  # # of cone pairs to publish ahead
        detected_cones = []
        for i in range(self.current_index, self.current_index + n_publish_ahead):
            detected_cones.append(cone_msg(self.blue_cones[i][0], self.blue_cones[i][1], Cone.BLUE))
            detected_cones.append(cone_msg(self.yellow_cones[i][0], self.yellow_cones[i][1], Cone.YELLOW))

        # Add cones to the message
        msg.cones = detected_cones

        # Publish cones for current_index
        self.publisher_.publish(msg)

        # Log cone publishing
        for cone in detected_cones:
            self.get_logger().info(f"Publishing Cone: x={cone.location.x}, y={cone.location.y}, color={cone.color}")

        # Publish car pose for current index (position and orientation)
        car_x, car_y = self.car_positions[self.current_index]
        car_orien = self.car_orientation[self.current_index]
        pose_msg(
            car_x, car_y, float(car_orien["w"]), float(car_orien["x"]), float(car_orien["y"]), float(car_orien["z"])
        )

        # 🖥️ PLOTTING UPDATE (Only Displays Published Cones)
        if self.plotting_enabled:
            self.ax.clear()

            # Extract coordinates of only the published cones
            blue_x, blue_y = [], []
            yellow_x, yellow_y = [], []

            for cone in detected_cones:
                if cone.color == Cone.BLUE:
                    blue_x.append(cone.location.x)
                    blue_y.append(cone.location.y)
                elif cone.color == Cone.YELLOW:
                    yellow_x.append(cone.location.x)
                    yellow_y.append(cone.location.y)

            # Plot only the published cones
            self.ax.scatter(blue_x, blue_y, c="blue", label="Published Blue Cones")
            self.ax.scatter(yellow_x, yellow_y, c="yellow", label="Published Yellow Cones")

            # Plot current car position
            self.ax.scatter(car_x, car_y, c="black", s=50, label="Car Position")

            # Add car orientation (theta) as an arrow
            theta = self.car_orientation[self.current_index]["theta"]  # Extract orientation angle
            arrow_length = 5  # Scale for visibility
            dx = arrow_length * np.cos(theta)  # X-component of arrow
            dy = arrow_length * np.sin(theta)  # Y-component of arrow

            self.ax.quiver(
                car_x, car_y, dx, dy, angles="xy", scale_units="xy", scale=0.4, color="red", label="Car Orientation"
            )

            # Update plot
            self.ax.set_xlim(-20, 80)
            self.ax.set_ylim(-60, 60)
            self.ax.legend(["Blue Cones", "Yellow Cones", "Car Position", "Car Orientation"])
            self.ax.grid(True)
            plt.draw()
            plt.pause(0.1)

        # Move to next position and reset upon full lap
        self.current_index += 1
        if self.current_index >= len(self.car_positions):  # Reset if last position is reached
            self.current_index = 0


def main(args=None):
    rclpy.init(args=args)
    node = DummyTrackPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
