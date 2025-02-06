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
        self.dtheta = np.pi / 10  # Angle increment

    def publish_dummy_track(self):
        msg = ConeDetectionStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # Define the track points
        # blue_cones = [
        #     {"x": self.a_blue * np.cos(self.theta + i * np.pi / 8), "y": self.b_blue * np.sin(self.theta + i * np.pi / 8)}
        #     for i in range(40)
        # ]
        # yellow_cones = [
        #     {"x": self.a_yellow * np.cos(self.theta + np.pi / 2 + i * np.pi / 8), "y": self.b_yellow * np.sin(self.theta + np.pi / 2 + i * np.pi / 8)}
        #     for i in range(40)
        # ]

        blue_cones_positions = [
            [9.990482215818577, 0.21809693682668],
            [9.063077870366499, 2.1130913087034973],
            [6.7559020761566035, 3.68638668405062],
            [3.4202014332566884, 4.698463103929542],
            [-0.4361938736533589, 4.995241107909289],
            [-4.226182617406994, 4.53153893518325],
            [-7.37277336810124, 3.3779510380783018],
            [-9.396926207859083, 1.7101007166283444],
            [-9.990482215818577, -0.21809693682667913],
            [-9.0630778703665, -2.1130913087034964],
            [-6.7559020761566035, -3.6863866840506194],
            [-3.4202014332566937, -4.698463103929541],
            [0.43619387365335766, -4.995241107909289],
            [4.226182617406996, -4.531538935183249],
            [7.372773368101239, -3.377951038078302],
            [9.396926207859082, -1.710100716628347],
            [9.990482215818577, 0.21809693682667852],
            [9.063077870366499, 2.1130913087034977],
            [6.755902076156604, 3.6863866840506194],
            [3.420201433256695, 4.698463103929541],
            [-0.4361938736533565, 4.995241107909289],
            [-4.226182617407003, 4.531538935183248],
            [-7.372773368101237, 3.3779510380783027],
            [-9.396926207859085, 1.7101007166283435],
            [-9.990482215818577, -0.21809693682668235],
            [-9.063077870366495, -2.1130913087035013],
            [-6.755902076156593, -3.6863866840506248],
            [-3.420201433256688, -4.698463103929543],
            [0.4361938736533641, -4.995241107909289],
            [4.2261826174070025, -4.531538935183248],
            [7.372773368101237, -3.3779510380783027],
            [9.396926207859083, -1.7101007166283442],
            [9.990482215818577, 0.21809693682668174],
            [9.063077870366495, 2.113091308703501],
            [6.755902076156594, 3.6863866840506243],
            [3.420201433256689, 4.698463103929542],
            [-0.4361938736533628, 4.995241107909289],
            [-4.226182617407001, 4.5315389351832485],
            [-7.372773368101236, 3.3779510380783035],
            [-9.396926207859083, 1.7101007166283446],
        ]

        yellow_cones_positions = [
            [-0.14539795788445298, 1.6650803693030964],
            [-1.4087275391356644, 1.510512978394417],
            [-2.4575911227004132, 1.125983679359434],
            [-3.132308735953028, 0.5700335722094482],
            [-3.330160738606193, -0.07269897894222638],
            [-3.021025956788834, -0.7043637695678322],
            [-2.251967358718868, -1.2287955613502064],
            [-1.1400671444188952, -1.5661543679765142],
            [0.14539795788445256, -1.6650803693030964],
            [1.4087275391356628, -1.5105129783944171],
            [2.457591122700413, -1.1259836793594342],
            [3.132308735953027, -0.5700335722094491],
            [3.330160738606193, 0.07269897894222617],
            [3.0210259567888333, 0.7043637695678326],
            [2.2519673587188684, 1.2287955613502064],
            [1.1400671444188983, 1.5661543679765135],
            [-0.14539795788445217, 1.6650803693030964],
            [-1.4087275391356624, 1.5105129783944171],
            [-2.457591122700413, 1.1259836793594344],
            [-3.1323087359530284, 0.5700335722094478],
            [-3.330160738606193, -0.07269897894222449],
            [-3.021025956788832, -0.7043637695678339],
            [-2.251967358718869, -1.2287955613502064],
            [-1.1400671444189014, -1.566154367976513],
            [0.1453979578844547, -1.6650803693030964],
            [1.4087275391356622, -1.5105129783944173],
            [2.4575911227004164, -1.1259836793594322],
            [3.132308735953028, -0.5700335722094481],
            [3.330160738606193, 0.07269897894222428],
            [3.021025956788832, 0.7043637695678336],
            [2.251967358718869, 1.2287955613502062],
            [1.140067144418902, 1.566154367976513],
            [-0.14539795788445428, 1.6650803693030964],
            [-1.4087275391356617, 1.5105129783944173],
            [-2.4575911227004164, 1.1259836793594322],
            [-3.132308735953028, 0.5700335722094483],
            [-3.330160738606193, -0.07269897894222409],
            [-3.0210259567888325, -0.7043637695678334],
            [-2.2519673587188693, -1.228795561350206],
            [-1.1400671444189023, -1.5661543679765129],
        ]

        # blue_cones_positions = [
        #     [point["x"], point["y"]] for point in blue_cones
        #     ]
        # yellow_cones_positions = [
        #     [point["x"], point["y"]] for point in yellow_cones
        #     ]

        points = [
            [12.062088012695312, 1.751],
            [15.124588012695314, 1.753],
            [3.812088012695313, 1.754],
            [18.499588012695312, 1.755],
            [9.624588012695312, 1.756],
            [12.499588012695314, -1.757],
            [14.999588012695314, -1.758],
            [19.124588012695316, -1.759],
            [4.187088012695313, -1.751],
            [9.374588012695312, -1.752],
            [6.187088012695313, 1.753],
            [6.3745880126953125, -1.754],
            [5.6245880126953125, 1.755],
            [5.9370880126953125, -1.756],
        ]

        # self.get_logger().info(f"Blue_cone: {yellow_cones_positions}")
        # self.get_logger().info(f"Blue_cone: {blue_cones_positions}")
        # Set list of points equal to cone locations (spoof cone locations with pre-track list)

        for point in blue_cones_positions:
            cone = Cone()
            cone.location.x = point[0]
            cone.location.y = point[1]
            cone.color = Cone.BLUE
            msg.cones.append(cone)

        for point in yellow_cones_positions:
            cone = Cone()
            cone.location.x = point[0]
            cone.location.y = point[1]
            cone.color = Cone.YELLOW
            msg.cones.append(cone)

        # Publish nav_sim status
        nav_sim_msg = Bool()
        nav_sim_msg.data = True
        self.nav_sim_publisher_.publish(nav_sim_msg)

        # Add blue cones
        # for point in blue_cones:
        #     cone = Cone()
        #     cone.location.x = point["x"]
        #     cone.location.y = point["y"]
        #     cone.color = Cone.BLUE
        #     msg.cones.append(cone)

        # # Add yellow cones
        # for point in yellow_cones:
        #     cone = Cone()
        #     cone.location.x = point["x"]
        #     cone.location.y = point["y"]
        #     cone.color = Cone.YELLOW
        #     msg.cones.append(cone)

        self.publisher_.publish(msg)
        # self.get_logger().info(f"Published {msg} cones")

        # Publish car position and orientation
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = self.a_blue * np.cos(self.theta)
        pose_msg.pose.position.y = self.b_blue * np.sin(self.theta)
        pose_msg.pose.orientation.z = np.sin(self.theta / 2)
        pose_msg.pose.orientation.w = np.cos(self.theta / 2)
        self.pose_publisher_.publish(pose_msg)
        # self.get_logger().info(f"Publishing Car Location:\n x:{pose_msg.pose.position.x} y: {pose_msg.pose.position.y}")
        # self.get_logger().info(f"Publishing Car Orientation:\n z:{pose_msg.pose.orientation.z} w: {pose_msg.pose.orientation.w}")

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
