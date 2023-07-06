from math import cos, sin
import time

import cv2
import numpy as np
from transforms3d.euler import quat2euler

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import PathStamped
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, UInt8

from driverless_common.common import angle, dist, fast_dist, wrap_to_pi

from . import qos_profile

from typing import List

cv_bridge = CvBridge()
WIDTH = 1000
HEIGHT = 1000


class PurePursuit(Node):
    path = np.array([])
    count = 0
    following = False
    r2d = False
    fallback_path_points_offset = 0
    cog2axle = 0.5  # could be a declared parameter

    def __init__(self, node_name: str = "pure_pursuit_node"):
        super().__init__(node_name)

        # subscribers
        self.create_subscription(Bool, "/system/r2d", self.r2d_callback, 10)
        self.create_subscription(UInt8, "/system/laps_completed", self.lap_callback, 10)
        self.create_subscription(PathStamped, "/planner/path", self.path_callback, 10)
        # sync subscribers pose + velocity
        self.create_subscription(PoseWithCovarianceStamped, "/slam/car_pose", self.callback, qos_profile=qos_profile)

        # publishers
        self.control_publisher: Publisher = self.create_publisher(AckermannDriveStamped, "/control/driving_command", 10)
        self.debug_publisher: Publisher = self.create_publisher(Image, "/debug_imgs/pursuit_img", 1)

        # parameters
        self.Kp_ang = self.declare_parameter("Kp_ang", -3.0).value
        self.lookahead = self.declare_parameter("lookahead", 3.0).value
        self.vel_max = self.declare_parameter("vel_max", 5.0).value
        self.DEBUG_IMG = self.declare_parameter("debug_img", True).value

        if node_name == "pure_pursuit_node":
            self.get_logger().info("---Pure pursuit follower initalised---")

    def r2d_callback(self, msg: Bool):
        if msg.data:
            self.r2d = True
            self.get_logger().info("Ready to drive")
        else:
            self.r2d = False
            self.get_logger().info("Driving disabled")

    def lap_callback(self, msg: UInt8):
        if msg.data > 0:
            self.following = True
            self.get_logger().info("Lap completed, following commencing")
        else:
            self.following = False

    def get_wheel_position(self, pose: Pose) -> List[float]:
        """
        Gets the position of the steering axle from the car's center of gravity and heading
        * param pose: Pose msg of the car's center of gravity
        * return: [x,y,th] position of steering axle
        """
        # i, j, k angles in rad
        heading = quat2euler(
            [
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
            ]
        )[2]

        x_axle = pose.position.x + cos(heading) * self.cog2axle
        y_axle = pose.position.y + sin(heading) * self.cog2axle
        return [x_axle, y_axle, heading]

    def get_rvwp(self, car_pos: List[float]):
        """
        Retrieve angle between two points
        * param car_pos: [x,y,theta] pose of the car
        * param path: [[x0,y0,i0],[x1,y1,i1],...,[xn-1,yn-1,in-1]] path points
        * param rvwp_lookahead: distance to look ahead for the RVWP
        * return: RVWP position as [x,y,i]
        """
        # find the closest point on the path to the car
        close_dist = float("inf")
        close_index = None
        for i, p in enumerate(self.path):
            distance = fast_dist(p, car_pos)
            if distance < close_dist:
                close_dist = distance
                close_index = i
        close = self.path[close_index] if close_index is not None else car_pos
        if close_index is None:
            self.get_logger().warn("Could not find closest point, have used car's axle pos")

        # find the first point on the path that is further than the lookahead distance
        rvwp_dist = float("inf")
        rvwp_index = None
        for i, p in enumerate(self.path):
            distance = fast_dist(p, close)
            if distance <= self.lookahead**2 or distance >= rvwp_dist:
                continue

            # get angle to check if the point is in front of the car
            ang = angle(close, p)
            error = wrap_to_pi(car_pos[2] - ang)
            if np.pi / 2 > error > -np.pi / 2:
                rvwp_dist = distance
                rvwp_index = i

        if rvwp_index is None or rvwp_index == close_index:
            self.get_logger().warn("No valid RVWP found, using fallback point")
            path_points_count = len(self.path) - 1
            fallback_point = close_index + self.fallback_path_points_offset
            if fallback_point > path_points_count:
                rvwp_index = abs(path_points_count - fallback_point)
            else:
                rvwp_index = fallback_point

        return self.path[rvwp_index]

    def path_callback(self, spline_path_msg: PathStamped):
        # convert List[PathPoint] to 2D numpy array
        self.get_logger().debug(f"Spline Path Recieved - length: {len(spline_path_msg.path)}")
        self.path = np.array([[p.location.x, p.location.y, p.turn_intensity] for p in spline_path_msg.path])
        distance = 0
        for i in range(len(self.path) - 1):
            prev = self.path[i]
            curr = self.path[i + 1]
            distance += dist(prev, curr)
        # Calculate the number of path points to skip when finding a fallback RVWP.
        # Formula is the number of path points divided by the calculated distance in metres, giving the number of
        # points per metre, which is then multiplied by the lookahead value (also in metres) giving the number of
        # path points that should be skipped.
        self.fallback_path_points_offset = int(round(len(self.path) / distance * self.lookahead))

    def callback(self, msg: PoseWithCovarianceStamped):
        # Only start once the path has been recieved, it's a following lap, and we are ready to drive
        if not self.following or not self.r2d or self.path.size == 0:
            return

        start_time = time.time()

        # get the position of the center of gravity
        pose: List[float] = self.get_wheel_position(msg.pose.pose)

        # rvwp control
        rvwp: List[float] = self.get_rvwp(pose)

        des_heading_ang = angle(pose[:2], [rvwp[0], rvwp[1]])
        error = wrap_to_pi(pose[2] - des_heading_ang)
        steering_angle = np.rad2deg(error) * self.Kp_ang
        target_vel = self.vel_max

        if self.DEBUG_IMG:
            debug_img = self.draw_rvwp(rvwp, pose[:2], steering_angle, target_vel)[0]
            self.debug_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))

        # publish message
        control_msg = AckermannDriveStamped()
        control_msg.drive.steering_angle = steering_angle
        control_msg.drive.speed = target_vel
        self.control_publisher.publish(control_msg)

        self.count += 1
        if self.count == 50:
            self.count = 0
            self.get_logger().info(f"{(time.time() - start_time) * 1000}")

    def draw_rvwp(self, rvwp: List[float], position: List[float], steering_angle: float, velocity: float):
        # get dimensions of the path
        path_x_min = np.min(self.path[:, 0])
        path_x_max = np.max(self.path[:, 0])
        path_y_min = np.min(self.path[:, 1])
        path_y_max = np.max(self.path[:, 1])

        # scale the path to fit in a 1000x1000 image, pixels per meter
        scale = WIDTH / max(path_x_max - path_x_min, path_y_max - path_y_min)

        # add a border around the path for all elements
        scale *= 0.90

        # set offsets to the corner of the image
        x_offset = -path_x_min * scale
        x_offset += (WIDTH - (path_x_max - path_x_min) * scale) / 2
        y_offset = -path_y_min * scale
        y_offset += (HEIGHT - (path_y_max - path_y_min) * scale) / 2

        debug_img = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
        for i in range(0, len(self.path) - 1):
            cv2.line(
                debug_img,
                (
                    int(self.path[i, 0] * scale + x_offset),
                    int(self.path[i, 1] * scale + y_offset),
                ),
                (
                    int(self.path[i + 1, 0] * scale + x_offset),
                    int(self.path[i + 1, 1] * scale + y_offset),
                ),
                (255, 0, 0),
                thickness=5,
            )

        cv2.drawMarker(
            debug_img,
            (int(rvwp[0] * scale + x_offset), int(rvwp[1] * scale + y_offset)),
            (0, 255, 0),
            markerType=cv2.MARKER_TRIANGLE_UP,
            markerSize=10,
            thickness=4,
        )

        cv2.drawMarker(
            debug_img,
            (int(position[0] * scale + x_offset), int(position[1] * scale + y_offset)),
            (0, 0, 255),
            markerType=cv2.MARKER_TRIANGLE_UP,
            markerSize=10,
            thickness=4,
        )
        cv2.flip(debug_img, 1, debug_img)

        text_angle = "Steering: " + str(round(steering_angle, 2))
        cv2.putText(debug_img, text_angle, (10, HEIGHT - 75), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 4)
        text_vel = "Velocity: " + str(round(velocity, 2))
        cv2.putText(debug_img, text_vel, (10, HEIGHT - 25), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 4)

        return debug_img, scale, x_offset, y_offset


def main(args=None):  # begin ros node
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
