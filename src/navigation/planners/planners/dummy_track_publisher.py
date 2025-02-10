import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node

from driverless_msgs.msg import Cone, ConeDetectionStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool

from driverless_common.common import QOS_LATEST, angle, midpoint


class DummyTrackPublisher(Node):
    def __init__(self):
        super().__init__("dummy_track_publisher")
        self.publisher_ = self.create_publisher(ConeDetectionStamped, "slam/cone_detection", 10)
        self.nav_sim_publisher_ = self.create_publisher(Bool, "nav_sim", 10)
        self.pose_publisher_ = self.create_publisher(PoseStamped, "car/pose", 10)
        self.timer = self.create_timer(1 / 3, self.publish_dummy_track)  # Publish every second
        self.midline_path_sub = self.create_subscription(
            Path, "planning/midline_path", self.midline_path_callback, QOS_LATEST
        )

        # Plotting flag
        self.plotting_enabled = True

        # Variable Initialisations
        self.received_midline_path = False
        self.midline_path = []

        # Publish nav_sim status (tell node_ft_planner, we're simulating)
        nav_sim_msg = Bool()
        nav_sim_msg.data = True
        self.nav_sim_publisher_.publish(nav_sim_msg)

        self.blue_cones = [
            (0, 0),
            (0, 5),
            (0, 10),
            (0, 15),
            (0, 20),
            (0, 25),
            (0, 30),
            (0, 35),
            (0, 40),
            (0, 45),
            (0, 50),
            (0, 55),
            (0, 60),
            (0, 65),
            (0, 70),
            (0, 75),
            (0, 80),
            (0, 85),
            (0, 90),
            (0, 95),
            (0, 100),
            (0, 105),
            (0, 110),
            (0, 115),
            (0, 120),
            (0, 125),
            (0, 130),
            (0, 135),
            (0, 140),
            (0, 145),
        ]

        self.yellow_cones = [
            (4, 0),
            (4, 5),
            (4, 10),
            (4, 15),
            (4, 20),
            (4, 25),
            (4, 30),
            (4, 35),
            (4, 40),
            (4, 45),
            (4, 50),
            (4, 55),
            (4, 60),
            (4, 65),
            (4, 70),
            (4, 75),
            (4, 80),
            (4, 85),
            (4, 90),
            (4, 95),
            (4, 100),
            (4, 105),
            (4, 110),
            (4, 115),
            (4, 120),
            (4, 125),
            (4, 130),
            (4, 135),
            (4, 140),
            (4, 145),
        ]

        # Pre-Calculate car_position from the list of cones
        self.car_positions = []
        for i in range(len(self.yellow_cones)):
            x_mid = (self.yellow_cones[i][0] + self.blue_cones[i][0]) / 2
            y_mid = (self.yellow_cones[i][1] + self.blue_cones[i][1]) / 2
            self.car_positions.append((x_mid, y_mid))

        # Pre-Calculate the cars_orientation as quaternions (convert to euler in node_ft_planner)
        self.car_orientation = []
        for i in range(len(self.yellow_cones) - 1):
            orientation = self.calc_orien(self.car_positions[i], self.car_positions[i + 1])
            self.car_orientation.append(orientation)

        # Initialize plot (if plotting is enabled)
        if self.plotting_enabled:
            self.fig, self.ax = plt.subplots(figsize=(10, 6))

            # self.ax.set_xlim(-20, 80) # track #1
            # self.ax.set_ylim(-60, 60)
            self.ax.set_xlim(-40, 40)  # track #2
            self.ax.set_ylim(-10, 140)
            self.ax.set_title("Cone and Car Position Visualization")
            self.ax.set_xlabel("X")
            self.ax.set_ylabel("Y")

            self.ax.legend(["Blue Cones", "Yellow Cones", "Car Position"])
            self.ax.grid(True)

            plt.ion()  # Turn on interactive mode
            plt.show()

        # Initialize index for incremental publishing
        self.current_index = 0

    # Subscribe to the planning/midline_path to take back the planned path from node_ft_planner
    # NOTE: The car's position is being subscribed to here from node_ft_planner, this way the publisher
    # NOTE: follows the actual path planner rather than a pre-set list of mid-points in the pre-set track.
    def midline_path_callback(self, msg):
        self.received_midline_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.received_midline_path = True
        self.get_logger().info("Received Midline Path!")

    # Calc-Orientation
    def calc_orien(self, current_position, next_position):
        dx = next_position[0] - current_position[0]
        dy = next_position[1] - current_position[1]
        theta = np.arctan2(dy, dx)
        # Convert yaw to quaternion (only rotation about the Z-axis)
        w = np.cos(theta / 2)
        x = 0
        y = 0
        z = np.sin(theta / 2)
        return {"theta": theta, "w": w, "x": x, "y": y, "z": z}

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
                f"Publishing Car Position:\nx={self.car_positions[self.current_index][0]}, y={self.car_positions[self.current_index][1]}"
            )

        ##### PUBLISHING #####
        # Publish cones from cone list, x cones ahead of current_index (using publishing functions)
        # NOTE: In any of the planning scripts, cone locations are not retained, nor does the path planner retain the cone locations.
        # NOTE: The car path plans based on the current list of cones and car pose.
        n_publish_ahead = 5  # # of cone pairs to publish ahead
        detected_cones = []
        for i in range(self.current_index, self.current_index + n_publish_ahead):
            index = i % len(self.blue_cones)
            detected_cones.append(cone_msg(self.blue_cones[index][0], self.blue_cones[index][1], Cone.BLUE))
            detected_cones.append(cone_msg(self.yellow_cones[index][0], self.yellow_cones[index][1], Cone.YELLOW))

        # Add cones to the message
        msg.cones = detected_cones

        # Publish cones for current_index
        self.publisher_.publish(msg)

        # Log cone publishing
        for cone in detected_cones:
            self.get_logger().info(f"Publishing Cone:\nx={cone.location.x}, y={cone.location.y}, color={cone.color}")

        # Publish car pose for current index (either from list or from the planned path subscribed from node_ft_planner)
        if self.received_midline_path and len(self.midline_path) > 1:
            car_x, car_y = self.received_midline_path[0]
            car_orien = self.calc_orien(self.received_midline_path[0], self.received_midline_path[1])
            pose_msg(
                car_x, car_y, float(car_orien["w"]), float(car_orien["x"]), float(car_orien["y"]), float(car_orien["z"])
            )
        else:
            car_x, car_y = self.car_positions[self.current_index]
            if self.current_index < len(self.car_positions) - 1:  # orientation is 1 less length than position
                car_orien = self.car_orientation[self.current_index]
            else:
                car_orien = {"theta": 0, "w": 1, "x": 0, "y": 0, "z": 0}
            pose_msg(
                car_x, car_y, float(car_orien["w"]), float(car_orien["x"]), float(car_orien["y"]), float(car_orien["z"])
            )

        # 🖥️ PLOTTING UPDATE (Only Displays Published Cones)
        if self.plotting_enabled:
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
            self.ax.scatter(blue_x, blue_y, c="blue", s=40, label="Published Blue Cones")
            self.ax.scatter(yellow_x, yellow_y, c="yellow", s=40, label="Published Yellow Cones")

            # Plot current car position
            self.ax.scatter(car_x, car_y, c="black", s=40, label="Car Position")

            # Add car orientation (theta) as an arrow
            if self.current_index < len(self.car_positions) - 1:
                theta = self.car_orientation[self.current_index]["theta"]  # Extract orientation angle
                arrow_length = 4  # Scale for visibility
                dx = arrow_length * np.cos(theta)  # X-component of arrow
                dy = arrow_length * np.sin(theta)  # Y-component of arrow
                self.ax.quiver(
                    car_x, car_y, dx, dy, angles="xy", scale_units="xy", scale=0.3, color="red", label="Car Orientation"
                )

            # Update plot
            self.ax.set_xlim(-40, 40)  # track #1
            self.ax.set_ylim(-10, 140)
            # self.ax.set_xlim(-20, 80) # track #2
            # self.ax.set_ylim(-60, 60)
            self.ax.set_title("Dummy Track Publication")
            # self.ax.legend(["Blue Cones", "Yellow Cones", "Car Position", "Car Orientation"])
            handles, labels = self.ax.get_legend_handles_labels()
            self.ax.legend(handles, labels)
            self.ax.grid(True)
            plt.draw()
            plt.pause(0.1)

        # Move to next position and reset upon full lap
        self.current_index += 1
        # self.get_logger().info(f"current_index: {self.current_index}")
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
