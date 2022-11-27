from math import atan2, cos, sin, sqrt

import numpy as np
import scipy.spatial
from transforms3d.euler import quat2euler

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import PathStamped, Reset
from geometry_msgs.msg import PoseWithCovarianceStamped

from typing import List, Tuple

LOOKAHEAD = 6
LOOKAHEAD_VEL = 6


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


def get_RVWP(car_pos: List[float], path: np.ndarray, lookahead: float) -> np.ndarray:
    """
    Retrieve angle between two points
    * param car_pos: [x,y] coords of point 1
    * param path: [[x0,y0,i0],[x1,y1,i1],...,[xn-1,yn-1,in-1]] path points
    * param rvwp_lookahead: distance to look ahead for the RVWP
    * return: RVWP position as [x,y,i]
    """
    pos = np.array([[car_pos[0], car_pos[1]]])
    dists: np.ndarray = scipy.spatial.distance.cdist(
        path[:, :2],  # search all path points for x,y cols up to 3rd col (intensity)
        pos,
        "euclidean",
    )
    min_index: int = np.where(dists == np.amin(dists))[0][0]

    # vwp_dists: np.ndarray = scipy.spatial.distance.cdist(
    #     path[:, :2],  # search all path points for x,y cols up to 3rd col (intensity)
    #     [path[min_index, :2]],
    #     "euclidean",
    # )
    # vwp_dists = vwp_dists[vwp_dists > lookahead]
    # rvwp = path[np.where(vwp_dists == np.amin(vp_dists))[0][0]]

    if min_index + lookahead >= len(path):
        rvwp_index: int = len(path) - 1
    else:
        rvwp_index: int = min_index + lookahead

    print("path indices: ", len(path), "rvwp_index: ", rvwp_index, "min_index: ", min_index)
    rvwp: np.ndarray = path[rvwp_index]

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
    return (angle + np.pi) % (2 * np.pi) - np.pi


class PurePursuit(Node):
    path = np.array([])
    Kp_ang: float = 15
    Kp_vel: float = 0.08
    vel_max: float = 4  # m/s
    vel_min: float = 3  # m/s
    throttle_max: float = 0.2
    brake_max: float = 0.12
    Kp_brake: float = 0.0
    r2d: bool = True  # for reset

    def __init__(self):
        super().__init__("pure_pursuit")

        self.create_subscription(PathStamped, "/planner/path", self.path_callback, 10)
        # sync subscribers pose + velocity
        self.create_subscription(PoseWithCovarianceStamped, "/slam/car_pose", self.callback, 10)

        self.reset_sub = self.create_subscription(Reset, "/reset", self.reset_callback, 10)

        # publishers
        self.control_publisher: Publisher = self.create_publisher(AckermannDriveStamped, "/driving_command", 10)

        self.get_logger().info("---Path Follower Node Initalised---")
        self.get_logger().info("---Awaing Ready to Drive command *OVERRIDDEN*---")

    def reset_callback(self, reset_msg: Reset):
        self.path = np.array([])
        self.r2d = True

    def path_callback(self, spline_path_msg: PathStamped):
        # convert List[PathPoint] to 2D numpy array
        self.path = np.array([[p.location.x, p.location.y, p.turn_intensity] for p in spline_path_msg.path])
        self.get_logger().debug(f"Spline Path Recieved - length: {len(self.path)}")

    def callback(self, msg: PoseWithCovarianceStamped):
        if not self.r2d:
            return

        # Only start once the path has been recieved
        if self.path.size == 0:
            return

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
        rvwp: List[float] = get_RVWP(position, self.path, LOOKAHEAD)
        # steering control
        des_heading_ang = angle(position, [rvwp[0], rvwp[1]])
        steering_angle = wrap_to_pi(theta - des_heading_ang) * self.Kp_ang

        # velocity control
        rvwp: List[float] = get_RVWP(position, self.path, LOOKAHEAD_VEL)
        intensity = rvwp[2]

        # target velocity proportional to angle
        target_vel: float = self.vel_max - intensity * self.Kp_vel
        if target_vel < self.vel_min:
            target_vel = self.vel_min

        # publish message
        control_msg = AckermannDriveStamped()
        control_msg.drive.steering_angle = -steering_angle
        control_msg.drive.speed = target_vel

        self.control_publisher.publish(control_msg)


def main(args=None):  # begin ros node
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
