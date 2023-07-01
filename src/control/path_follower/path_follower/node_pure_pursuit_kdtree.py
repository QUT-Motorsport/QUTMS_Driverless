from math import cos, sin
import time

import numpy as np
from sklearn.neighbors import KDTree
from transforms3d.euler import quat2euler

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import PathStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image

from driverless_common.common import QOS_LATEST, angle, dist, wrap_to_pi

from . import qos_profile

from typing import List

cv_bridge = CvBridge()
WIDTH = 1000
HEIGHT = 1000


def get_wheel_position(pos_cog: List[float], heading: float) -> List[float]:
    """
    Gets the position of the steering axle from the car's center of gravity and heading
    * param pos_cog: [x,y] coords of the car's center of gravity
    * param heading: car's heading in rads
    * return: [x,y] position of steering axle
    """
    cog2axle = 0.5  # m
    x_axle = pos_cog[0] + cos(heading) * cog2axle
    y_axle = pos_cog[1] + sin(heading) * cog2axle

    return [x_axle, y_axle, heading]


class FastPurePursuit(Node):
    path = np.array([])
    count = 0
    fallback_path_points_offset = 0

    def __init__(self):
        super().__init__("fast_pure_pursuit_node")

        self.create_subscription(PathStamped, "/planner/path", self.path_callback, QOS_LATEST)
        # sync subscribers pose + velocity
        self.create_subscription(PoseWithCovarianceStamped, "/slam/car_pose", self.callback, QOS_LATEST)

        # publishers
        self.control_publisher: Publisher = self.create_publisher(AckermannDriveStamped, "/control/driving_command", 10)
        self.debug_publisher: Publisher = self.create_publisher(Image, "/debug_imgs/pursuit_img", 1)

        # parameters
        self.Kp_ang = self.declare_parameter("Kp_ang", -3.0).value
        self.lookahead = self.declare_parameter("lookahead", 3.0).value
        self.vel_max = self.declare_parameter("vel_max", 7.0).value
        self.DEBUG_IMG = self.declare_parameter("debug_img", True).value

        self.get_logger().info("---Path Follower Node Initalised---")

    def get_rvwp(self, car_pos: List[float]):
        """
        Retrieve angle between two points
        * param car_pos: [x,y,theta] pose of the car
        * param path: [[x0,y0,i0],[x1,y1,i1],...,[xn-1,yn-1,in-1]] path points
        * param rvwp_lookahead: distance to look ahead for the RVWP
        * return: RVWP position as [x,y,i]
        """
        path_xy = [[p[0], p[1]] for p in self.path]

        # find the closest point on the path to the car
        kdtree = KDTree(path_xy)
        close_index = kdtree.query([[car_pos[0], car_pos[1]]], return_distance=False)[0][0]
        close = path_xy[close_index] if close_index is not None else car_pos
        if close_index is None:
            self.get_logger().warn("Could not find closest point, have used car's axle pos")

        # find the first point on the path that is further than the lookahead distance
        # Tragically, there is no way to set a minimum search distance, so I'm giving it the lookahead doubled, we'll
        # receive everything under that distance and filter out the items under the lookahead distance below.
        # Problem there is if there are no points under the double lookahead distance, we're in trouble.
        indexes_raw, distances_raw = kdtree.query_radius(
            [close], r=self.lookahead * 2, return_distance=True, sort_results=True
        )
        indexes_raw, distances_raw = indexes_raw[0], distances_raw[0]

        indexes, distances = [], []
        for i in range(len(indexes_raw)):
            if distances_raw[i] < self.lookahead:
                continue
            # Distances are sorted, so once we get here just grab everything.
            indexes = indexes_raw[i:]
            distances = distances_raw[i:]
            break

        rvwp_dist = float("inf")
        rvwp_index = None
        for i in range(len(indexes)):
            index = indexes[i]
            p = path_xy[index]
            d = distances[i]

            # get angle to check if the point is in front of the car
            ang = angle(close, p)
            error = wrap_to_pi(car_pos[2] - ang)
            if np.pi / 2 > error > -np.pi / 2 and d < rvwp_dist:
                rvwp_dist = d
                rvwp_index = index

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
        # Only start once the path has been recieved
        if self.path.size == 0:
            return
        start_time = time.time()

        # i, j, k angles in rad
        theta = quat2euler(
            [
                msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
            ]
        )[2]
        # get the position of the center of gravity
        position_cog: List[float] = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        position: List[float] = get_wheel_position(position_cog, theta)

        # rvwp control
        rvwp: List[float] = self.get_rvwp(position)

        des_heading_ang = angle(position, [rvwp[0], rvwp[1]])
        error = wrap_to_pi(theta - des_heading_ang)
        steering_angle = np.rad2deg(error) * self.Kp_ang

        # publish message
        control_msg = AckermannDriveStamped()
        control_msg.drive.steering_angle = steering_angle
        control_msg.drive.speed = self.vel_max
        self.control_publisher.publish(control_msg)

        self.count += 1
        if self.count == 50:
            self.count = 0
            self.get_logger().info(f"{(time.time() - start_time) * 1000}")


def main(args=None):  # begin ros node
    rclpy.init(args=args)
    node = FastPurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
