import math
import time

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
        self.timer = self.create_timer(1 / 15, self.publish_dummy_track)  # Publish every second
        self.midline_path_sub = self.create_subscription(
            Path, "planning/midline_path", self.midline_path_callback, QOS_LATEST
        )

        # Plotting flag
        self.plotting_enabled = True

        # Initialisation flag
        self.init = True

        # Variable Initialisations
        self.received_midline_path = False
        self.midline_path = []
        self.car_x = 0.0
        self.car_y = 0.0
        # Lists to store detected cones for plotting
        self.detected_blue_cones = []
        self.detected_yellow_cones = []

        # Publish nav_sim status (tell node_ft_planner, we're simulating)
        nav_sim_msg = Bool()
        nav_sim_msg.data = True
        self.nav_sim_publisher_.publish(nav_sim_msg)

        self.blue_cones = [
            [-0.688629150390625, 12.499588012695314],
            [-0.626129150390625, 14.999588012695314],
            [-0.313629150390625, 19.124588012695316],
            [-1.5011291503906252, 4.187088012695313],
            [-0.9386291503906252, 9.374588012695312],
            [0.24651104798296636, 22.383778126032734],
            [0.7488708496093751, 26.124588012695316],
            [1.123870849609371, 30.249588012695316],
            [2.811370849609375, 33.74958801269531],
            [4.979002109684964, 35.14119067330027],
            [8.561370849609375, 35.99958801269531],
            [12.686370849609375, 36.24958801269531],
            [14.882646048932394, 35.19033668603414],
            [17.123870849609375, 33.74958801269531],
            [19.623870849609375, 31.874588012695312],
            [21.248870849609375, 28.999588012695312],
            [22.811370849609375, 25.812088012695312],
            [23.811370849609375, 22.999588012695312],
            [24.436370849609375, 19.749588012695312],
            [24.873870849609375, 16.937088012695312],
            [24.954850285923115, 13.33911207231931],
            [24.186370849609375, 10.062088012695309],
            [21.811370849609375, 7.312088012695316],
            [19.643580836900586, 5.924854353936258],
            [17.748870849609375, 3.6245880126953143],
            [16.498870849609375, -0.0004119873046875],
            [17.062108458339143, -3.7020445098787143],
            [24.248870849609375, -7.5004119873046875],
            [21.279636416219446, -7.719703640396514],
            [18.395127341766017, -6.9416363879074865],
            [-1.376129150390625, -3.812911987304698],
            [-1.4386291503906214, -6.187911987304695],
            [15.311370849609377, -25.87541198730469],
            [18.657499362530505, -26.77208051261518],
            [32.311370849609375, -11.250411987304688],
            [35.436370849609375, -10.750411987304688],
            [29.285487862203865, -8.925515117731099],
            [37.400754660666784, -9.666254471191838],
            [39.373870849609375, -7.6879119873046875],
            [26.998870849609375, -7.8754119873046875],
            [40.623870849609375, -5.812911987304688],
            [38.873870849609375, -24.000411987304688],
            [42.998870849609375, -22.937911987304688],
            [46.186370849609375, -22.250411987304688],
            [49.186370849609375, -20.625411987304688],
            [52.186370849609375, -18.187911987304688],
            [54.998870849609375, -17.187911987304688],
            [43.686370849609375, 11.187088012695312],
            [41.873870849609375, 8.18708801269531],
            [41.561370849609375, 1.0620880126953125],
            [41.186370849609375, 4.312088012695311],
            [46.061370849609375, 13.124588012695312],
            [48.686370849609375, 13.937088012695312],
            [52.18637084960937, 14.124588012695312],
            [55.686370849609375, 13.562088012695312],
            [59.123870849609375, 11.937088012695314],
            [61.561370849609375, 10.312088012695312],
            [64.06137084960938, 6.3120880126953125],
            [65.43637084960938, 2.6245880126953125],
            [64.12387084960938, -1.0629119873046893],
            [62.748870849609375, -4.7504119873046875],
            [61.936370849609375, -7.187911987304686],
            [60.936370849609375, -9.000411987304688],
            [59.998870849609375, -11.812911987304691],
            [57.873870849609375, -15.250411987304688],
            [35.873870849609375, -25.125411987304688],
            [32.748870849609375, -25.937911987304695],
            [28.93637084960937, -26.750411987304688],
            [25.68637084960938, -26.937911987304695],
            [22.248870849609375, -27.125411987304688],
            [11.936370849609375, -24.875411987304688],
            [7.9988708496093786, -23.062911987304688],
            [5.2488708496093786, -21.062911987304688],
            [3.061370849609375, -18.937911987304688],
            [1.2488708496093786, -16.062911987304688],
            [-0.126129150390625, -12.937911987304688],
            [-1.376129150390625, -9.562911987304688],
            [-1.8761291503906268, -0.812911987304684],
            [-1.5636291503906268, 1.2495880126953125],
            [41.436370849609375, -2.6879119873046866],
        ]

        self.yellow_cones = [
            [2.0613708496093754, 12.062088012695312],
            [2.248870849609375, 15.124588012695314],
            [1.6238708496093752, 3.812088012695313],
            [2.811370849609375, 18.499588012695312],
            [1.9988708496093752, 9.624588012695312],
            [3.1238708496093706, 21.937088012695312],
            [3.5613708496093794, 25.37458801269531],
            [4.811370849609379, 29.249588012695312],
            [7.373870849609374, 31.874588012695312],
            [10.917827368100067, 32.70692136814713],
            [21.248870849609375, 17.499588012695312],
            [21.882050232110036, 14.499699134665917],
            [20.139784073194402, 20.734654217222435],
            [20.873870849609375, 11.562088012695312],
            [13.279457756758717, -1.1879920204041703],
            [13.498870849609373, 2.8745880126953107],
            [13.686370849609375, -5.1879119873046875],
            [15.623870849609375, 6.249588012695312],
            [14.936370849609375, -9.062911987304688],
            [1.561370849609375, 1.2495880126953125],
            [1.936370849609375, -3.8754119873046875],
            [1.623870849609375, -1.1879119873046875],
            [2.061370849609368, -6.37541198730468],
            [2.3113708496093803, -8.562911987304693],
            [2.498870849609382, -11.062911987304693],
            [5.748870849609377, -15.937911987304688],
            [3.8738708496093768, -13.750411987304686],
            [7.796020029194144, -18.254126180161492],
            [18.122405765055735, -11.939883241835924],
            [13.248870849609375, -21.000411987304688],
            [10.998870849609375, -20.125411987304688],
            [15.804603853839973, -21.978172248794852],
            [18.324983793858365, -22.939430941846098],
            [19.2, 9],
            [21.311370849609375, -23.68791198730469],
            [22.561370849609375, -11.625411987304688],
            [23.998870849609375, -23.68791198730469],
            [31.018338822668763, -14.506836514198806],
            [25.500240741576917, -11.304083820428891],
            [26.203246240018245, -23.56515670452681],
            [28.48064949143506, -12.721048422586975],
            [35.436370849609375, -14.062911987304688],
            [29.561370849609375, -23.125411987304688],
            [32.58648594658583, -22.91148372547888],
            [39.123870849609375, -12.625411987304688],
            [35.248870849609375, -21.937911987304688],
            [37.686370849609375, -21.062911987304688],
            [42.186370849609375, -10.062911987304688],
            [41.498870849609375, -20.68791198730469],
            [44.498870849609375, -6.062911987304686],
            [44.873870849609375, -19.437911987304688],
            [47.998870849609375, -17.062911987304688],
            [49.936370849609375, -15.375411987304688],
            [52.31794925819424, -14.454415017183544],
            [55.186370849609375, -12.812911987304688],
            [45.373870849609375, -2.6254119873046875],
            [56.936370849609375, -11.062911987304688],
            [58.123870849609375, -8.125411987304688],
            [45.248870849609375, 0.9370880126953125],
            [44.936370849609375, 4.4995880126953125],
            [45.998870849609375, 7.3120880126953125],
            [48.561370849609375, 8.937088012695312],
            [51.18637084960937, 9.749588012695312],
            [54.998870849609375, 9.499588012695314],
            [60.873870849609375, -0.5004119873046875],
            [59.561370849609375, -4.2504119873046875],
            [60.061370849609375, 5.8745880126953125],
            [60.917484762030114, 3.092754987622678],
        ]

        # Pre-Calculate initial car position & orientation from the list of cones
        self.initial_car_position = (
            (self.yellow_cones[0][0] + self.blue_cones[0][0]) / 2,
            (self.yellow_cones[0][1] + self.blue_cones[0][1]) / 2,
        )
        self.next_initial_car_position = (
            (self.yellow_cones[1][0] + self.blue_cones[1][0]) / 2,
            (self.yellow_cones[1][1] + self.blue_cones[1][1]) / 2,
        )
        self.initial_car_orientation = self.calc_orien(self.initial_car_position, self.next_initial_car_position)

        # Initialize plot (if plotting is enabled)
        if self.plotting_enabled:
            self.fig, self.ax = plt.subplots(figsize=(10, 6))

            # self.ax.set_xlim(-20, 80) # track #1
            # self.ax.set_ylim(-60, 60)
            self.ax.set_xlim(-10, 70)  # track #2
            self.ax.set_ylim(-30, 40)
            self.ax.set_title("Cone and Car Position Visualization")
            self.ax.set_xlabel("X")
            self.ax.set_ylabel("Y")

            # self.ax.legend(["Blue Cones", "Yellow Cones", "Car Position"])
            self.ax.grid(True)

            plt.ion()  # Turn on interactive mode
            plt.show()

        ######### END OF __init__() #########

    # Subscribe to the planning/midline_path to take back the planned path from node_ft_planner
    # NOTE: The car's position is being subscribed to here from node_ft_planner, this way the publisher
    # NOTE: follows the actual path planner rather than a pre-set list of mid-points in the pre-set track.
    def midline_path_callback(self, msg):
        self.midline_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
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

    # cone_msg : function used to create cone message to publish
    def create_cone_msg(self, cone_x, cone_y, cone_color):
        # Create Cone message
        cone = Cone()
        cone.location.x = float(cone_x)
        cone.location.y = float(cone_y)
        cone.location.z = 0.0  # Ensure z is set to 0.0
        cone.color = cone_color  # Set color (BLUE/YELLOW)

        if cone_color == Cone.BLUE:  # append persistent cone variables for plotting
            self.detected_blue_cones.append((cone_x, cone_y))
        elif cone_color == Cone.YELLOW:
            self.detected_yellow_cones.append((cone_x, cone_y))

        return cone

    # pose_msg : function to publish pose locations (car_orientation & car_position) as PoseStamped msg
    def pose_msg(self, car_x, car_y, car_orien_w, car_orien_x, car_orien_y, car_orien_z):
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
        self.get_logger().info(f"Publishing Initial Car Position!:\n {car_x, car_y}")

    def filter_coordinates(self, center_x, center_y, radius, coordinates):
        filtered_coords = []
        for x, y in coordinates:
            distance = math.sqrt((x - center_x) ** 2 + (y - center_y) ** 2)  # Calculate Euclidean distance
            if distance <= radius:  # Check if the point is within the given radius
                filtered_coords.append((x, y))  # Add to the result list
        return filtered_coords  # Return the filtered list

    def plot_cones(self, car_orien):
        if not self.plotting_enabled:
            return
        self.ax.clear()
        # Extract coordinates for plotting
        blue_x, blue_y = zip(*self.detected_blue_cones) if self.detected_blue_cones else ([], [])
        yellow_x, yellow_y = zip(*self.detected_yellow_cones) if self.detected_yellow_cones else ([], [])
        # Plot detected cones
        self.ax.scatter(blue_x, blue_y, c="blue", s=10, label="Published Blue Cones")
        self.ax.scatter(yellow_x, yellow_y, c="yellow", s=10, label="Published Yellow Cones")
        # Plot car position
        self.ax.scatter(self.car_x, self.car_y, c="black", s=20, label="Car Position")
        # Add car orientation as an arrow
        if self.received_midline_path and len(self.midline_path) > 1:
            theta = car_orien["theta"]
            arrow_length = 1
            dx = arrow_length * np.cos(theta)
            dy = arrow_length * np.sin(theta)
            self.ax.quiver(
                self.car_x,
                self.car_y,
                dx,
                dy,
                angles="xy",
                scale_units="xy",
                scale=0.2,
                color="red",
                label="Car Orientation",
            )
        # Update plot settings
        self.ax.set_xlim(-10, 70)
        self.ax.set_ylim(-30, 40)
        self.ax.set_title("Dummy Track Publication")
        self.ax.legend()
        self.ax.grid(True)
        # Refresh plot
        plt.draw()
        plt.pause(0.1)

    ##### MAIN TIMER CALLBACK ########
    def publish_dummy_track(self):
        if self.init:  # Upon initialisation, publish initial car_pose as midpoint between cones.
            self.car_x, self.car_y = self.initial_car_position
            self.pose_msg(
                self.initial_car_position[0],
                self.initial_car_position[1],
                float(self.initial_car_orientation["w"]),
                float(self.initial_car_orientation["x"]),
                float(self.initial_car_orientation["y"]),
                float(self.initial_car_orientation["z"]),
            )
            self.init = False

        #### CONE PUBLISHING ####
        msg = ConeDetectionStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.cones = []  # Store cones here, batch them together

        car_radius = 10  # Radius of the car's cone detection
        for coordinate in self.filter_coordinates(self.car_x, self.car_y, car_radius, self.blue_cones):
            msg.cones.append(self.create_cone_msg(coordinate[0], coordinate[1], Cone.BLUE))
        for coordinate in self.filter_coordinates(self.car_x, self.car_y, car_radius, self.yellow_cones):
            msg.cones.append(self.create_cone_msg(coordinate[0], coordinate[1], Cone.YELLOW))

        self.publisher_.publish(msg)  # Publish the batch
        self.get_logger().info(f"Published {len(msg.cones)} cones")

        # Upon recieving planned path, publish the next n waypoint within the midline_path as the next car_pose.
        if self.received_midline_path and len(self.midline_path) > 1:
            self.get_logger().info(f"recieved midline path! calculating & publishing new car_pose")
            self.car_x, self.car_y = self.midline_path[3]
            car_orien = self.calc_orien(self.midline_path[0], self.midline_path[3])
            self.pose_msg(
                self.car_x,
                self.car_y,
                float(car_orien["w"]),
                float(car_orien["x"]),
                float(car_orien["y"]),
                float(car_orien["z"]),
            )
        else:
            self.get_logger().warn("Not received midline path! Returning out of timer callback!")
            # self.get_logger().warn(f"Length of midline path: {len(self.midline_path)}")
            return

        self.plot_cones(car_orien)


def main(args=None):
    rclpy.init(args=args)
    node = DummyTrackPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
