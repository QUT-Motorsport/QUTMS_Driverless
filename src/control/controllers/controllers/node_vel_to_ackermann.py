from math import atan, pi

import numpy as np
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path


class Vel2Ackermann(Node):
    path = None

    def __init__(self):
        super().__init__("nav_cmd_translator")

        self.declare_parameter("Kp", 4.0)
        self.declare_parameter("wheelbase", 1.5)
        self.declare_parameter("distance_ctrl", True)
        self.declare_parameter("distance_Kp", -20.0)
        self.declare_parameter("distance_max", 1.5)
        self.declare_parameter("map_frame", "track")
        self.declare_parameter("base_frame", "base_footprint")
        self.Kp = self.get_parameter("Kp").value
        self.wheelbase = self.get_parameter("wheelbase").value
        self.distance_ctrl = self.get_parameter("distance_ctrl").value
        self.distance_Kp = self.get_parameter("distance_Kp").value
        self.distance_max = self.get_parameter("distance_max").value
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.create_subscription(Twist, "/control/nav_cmd_vel", self.cmd_callback, 1)

        if self.distance_ctrl:
            self.create_subscription(Path, "/planning/midline_path", self.path_callback, 1)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/control/driving_command", 1)

        self.get_logger().info("---Nav2 control interpreter initalised---")

    def cmd_callback(self, twist_msg: Twist):
        vel = twist_msg.linear.x

        if twist_msg.angular.z == 0 or vel == 0:
            steering = 0.0
        else:
            radius = vel / twist_msg.angular.z
            steering = atan(self.wheelbase / radius) * (180 / pi) * self.Kp  ## MAKE THIS A PARAM

        if self.distance_ctrl:
            steering = self.distance_control(steering)

        msg = AckermannDriveStamped()
        # make time for msg id
        # msg.header.stamp =
        msg.header.frame_id = "base_footprint"
        msg.drive.steering_angle = steering
        msg.drive.speed = vel

        self.drive_pub.publish(msg)

    def path_callback(self, path_msg: Path):
        self.path = path_msg

    def find_nearest_point(self, position: np.ndarray, path_points: np.ndarray):
        distances = [np.linalg.norm(position - point) for point in path_points]
        nearest_index = np.argmin(distances)
        return nearest_index, path_points[nearest_index]

    def compute_lateral_error(self, position: np.ndarray, path_points: np.ndarray):
        nearest_index, nearest_point = self.find_nearest_point(position, path_points)
        if nearest_index < len(path_points) - 1:
            next_point = path_points[nearest_index + 1]
        else:
            next_point = path_points[nearest_index - 1]
        
        tangent = next_point - nearest_point
        tangent /= np.linalg.norm(tangent)
        
        vector_to_robot = position - nearest_point
        lateral_error = np.cross(tangent, vector_to_robot)
        return lateral_error

    def distance_control(self, steering):
        try:
            # TODO: parameterise these frames?
            map_to_base = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn("Transform exception: " + str(e), throttle_duration_sec=1)
            return steering

        points = np.array([[point.pose.position.x, point.pose.position.y] for point in self.path.poses])
        distance = self.compute_lateral_error([map_to_base.transform.translation.x, map_to_base.transform.translation.y], points)

        # if distance is greater than max, set to max
        if distance > self.distance_max:
            distance = self.distance_max
        elif distance <= -self.distance_max:
            distance = -self.distance_max

        # calculate steering modifier
        modifier = distance * self.distance_Kp
        return steering + modifier

def main(args=None):
    # begin ros node
    rclpy.init(args=args)
    node = Vel2Ackermann()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
