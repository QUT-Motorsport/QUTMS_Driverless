from math import atan2, cos, sin, sqrt

import numpy as np
import scipy.spatial
from transforms3d.euler import quat2euler

import message_filters
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDrive
from driverless_msgs.msg import PathStamped
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped

from typing import List


def get_wheel_position(pos_cog: List[float], heading: float) -> List[float]:
    """
    Gets the position of the steering axle from the car's center of gravity and heading
    * param pos_cog: [x,y] coords of the car's center of gravity
    * param heading: car's heading in rads
    * return: [x,y] position of steering axle
    """
    # https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/v2.1.0/vehicle_model/
    cog2axle = 0.4  # m
    x_axle = pos_cog[0] + cos(heading) * cog2axle
    y_axle = pos_cog[1] + sin(heading) * cog2axle

    return [x_axle, y_axle]


def get_RVWP(car_pos: List[float], path: np.ndarray, rvwp_lookahead: int) -> List[float]:
    """
    Retrieve angle between two points
    * param car_pos: [x,y] coords of point 1
    * param path: [[x0,y0],[x1,y1],...,[xn-1,yn-1]] path points
    * param rvwpLookahead: how many indices to look ahead in path array for RVWP
    * return: RVWP position as [x,y]
    """
    _pos = np.array([[car_pos[0], car_pos[1]]])
    dists: np.ndarray = scipy.spatial.distance.cdist(path, _pos, "euclidean")
    min_index: int = np.where(dists == np.amin(dists))[0][0]

    rvwp_index: int = (min_index + rvwp_lookahead) % len(path)
    rvwp: List[float] = path[rvwp_index]

    return rvwp


def angle(p1: List[float], p2: List[float]) -> float:
    """
    Retrieve angle between two points
    * param p1: [x,y] coords of point 1
    * param p2: [x,y] coords of point 2
    * return: angle in rads
    """
    x_disp = p2[0] - p1[0]
    y_disp = p2[1] - p1[1]
    return atan2(y_disp, x_disp)


def wrap_to_pi(angle: float) -> float:
    """
    Wrap an angle between -pi and pi
    * param angle: angle in rads
    * return: angle in rads wrapped to -pi and pi
    """

    # https://stackoverflow.com/a/15927914/12206202
    return (angle + np.pi) % (2 * np.pi) - np.pi


class PurePursuit(Node):
    path: np.ndarray = []
    Kp_ang: float = 4.5
    Kp_vel: float = 4.5
    vel_max: float = 14  # m/s
    vel_min: float = 1.0  # m/s
    throttle_max: float = 0.5
    brake_max: float = 0.15

    def __init__(self):
        super().__init__("pure_pursuit")

        # sub to path mapper for the desired vehicle path (as an array)
        self.create_subscription(PathStamped, "/path_planner/path", self.path_callback, 10)
        # sync subscribers pose + velocity
        pose_sub = message_filters.Subscriber(self, PoseWithCovarianceStamped, "/zed2i/zed_node/pose_with_covariance")
        vel_sub = message_filters.Subscriber(self, TwistWithCovarianceStamped, "/imu/velocity")
        synchronizer = message_filters.ApproximateTimeSynchronizer(fs=[pose_sub, vel_sub], queue_size=20, slop=0.1)
        synchronizer.registerCallback(self.callback)

        # publishers
        self.control_publisher: Publisher = self.create_publisher(AckermannDrive, "/driving_command", 10)

        self.get_logger().info("---Path Follower Node Initalised---")

    def path_callback(self, spline_path_msg: PathStamped):
        # Only set the desired path once (before the car is moving)
        if self.path != []:
            return

        # convert List[PathPoint] to 2D numpy array
        self.path = np.array([[p.location.x, p.location.y] for p in spline_path_msg.path])
        self.get_logger().debug(f"Spline Path Recieved - length: {len(self.path)}")

    def callback(
        self,
        pose_msg: PoseWithCovarianceStamped,
        vel_msg: TwistWithCovarianceStamped,
    ):
        # Only start once the path has been recieved
        if self.path == []:
            return

        # i, j, k angles in rad
        ai, aj, ak = quat2euler(
            [
                pose_msg.pose.pose.orientation.w,
                pose_msg.pose.pose.orientation.x,
                pose_msg.pose.pose.orientation.y,
                pose_msg.pose.pose.orientation.z,
            ]
        )
        # get the position of the center of gravity
        position_cog: List[float] = [pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y]
        position: List[float] = get_wheel_position(position_cog, ak)

        # rvwp control
        rvwpLookahead = 70
        rvwp: List[float] = get_RVWP(position, self.path, rvwpLookahead)

        # steering control
        des_heading_ang = angle(position, rvwp)
        steering_angle = wrap_to_pi(ak - des_heading_ang) * self.Kp_ang

        # velocity control
        vel = sqrt(vel_msg.twist.twist.linear.x**2 + vel_msg.twist.twist.linear.y**2)
        # target velocity proportional to angle
        target_vel: float = self.vel_max - abs(steering_angle) * self.Kp_vel
        if target_vel < self.vel_min:
            target_vel = self.vel_min

        # increase proportionally as it approaches target
        throttle_scalar: float = 1 - (vel / target_vel)
        calc_brake = 0.0
        if throttle_scalar > 0:
            calc_throttle = self.throttle_max * throttle_scalar
        # if its over maximum, brake propotionally unless under minimum
        else:
            calc_throttle = 0.0
            if vel > self.vel_min:
                calc_brake = abs(self.brake_max * throttle_scalar)

        # publish message
        control_msg = AckermannDrive()
        control_msg.steering_angle = steering_angle
        control_msg.acceleration = calc_throttle
        control_msg.jerk = calc_brake  # using jerk for brake for now

        self.control_publisher.publish(control_msg)


def main(args=None):  # begin ros node
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
