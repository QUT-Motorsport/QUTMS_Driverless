from math import atan2, cos, dist, pi, sin, sqrt
import time

import cv2
import numpy as np
import scipy.spatial
from transforms3d.euler import quat2euler

from cv_bridge import CvBridge
import rclpy

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import Cone, ConeDetectionStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

from driverless_common.common import angle, fast_dist, wrap_to_pi
from path_follower.node_pure_pursuit import PurePursuit

from typing import List

# ADD QOS PROFILES

LEFT_CONE_COLOUR = Cone.BLUE
RIGHT_CONE_COLOUR = Cone.YELLOW

cv_bridge = CvBridge()
WIDTH = 1000
HEIGHT = 1000


def get_closest_cone(pos_car: List[float], boundaries: np.ndarray) -> float:
    """
    Gets the position of the nearest cone to the car.
    * param pos_car: [x,y] coords of car position
    * param boundaries: [x,y] coords of all current cones
    * return: [x,y] of nearest cone to the car
    """

    # get arrays of only coords for more efficient compute
    _pos = np.array([[pos_car[0], pos_car[1]]])
    boundaries_coords = np.array(boundaries[:, :2])

    # find distances of all cones and index of closest cone (improve by finding distances of close cones only?)
    dists: np.ndarray = scipy.spatial.distance.cdist(
        boundaries_coords,
        _pos,
        "euclidean",
    )
    # not certain np.where is returning the index - it should though
    nearest_cone_index: int = np.where(dists == np.amin(dists))[0][0]

    nearest_cone = boundaries_coords[nearest_cone_index]
    return nearest_cone


class ParticlePursuit(PurePursuit):
    """
    This inherits from PurePursuit as the underlying path following algorithm is the same

    Avoidance adapted from: https://link.springer.com/chapter/10.1007/978-3-031-10047-5_5
    * Treats the vehicle as a charged point particle, interacting \
    * with two external charged forces (barrier - repulsive, lookahead - attractive).
    """

    # ------------------------------
    track = np.array([])

    # attractive force constants:
    k_attractive: float = 3  # attractive force gain

    # repulsive force constants:
    d_min: float = 1.3  # min repulsive force distance (max. repulsion at or below)
    d_max: float = 2.0  # max repulsive force distance (zero repulsion at or above)
    k_repulsive: float = 0  # repulsive force gain

    # cone_danger - a unitless, *inverse* 'spring constant' of the repulsive force (gamma in documentation)
    # E.g. cone_danger > 0: corners cut tighter
    #      cone_danger < 0: corners taken wider
    #      ** dont set to 1.0 **
    cone_danger: float = 0.0

    def __init__(self):
        super().__init__("particle_pursuit")

        # sub to track for all cone locations relative to car start point, used for boundary danger calculations
        self.create_subscription(ConeDetectionStamped, "/planner/interpolated_map", self.interp_track_callback, 10)

        self.get_logger().info("---Particle pursuit follower initalised---")

    # recieve the cone locations
    def interp_track_callback(self, cone_pos_msg: ConeDetectionStamped):
        self.get_logger().debug("Map received")

        ## MOVED FROM CALLBACK AS THIS ONLY HAS TO HAPPEN ONCE
        # get left and right cones
        left_cones = [c for c in cone_pos_msg.cones if c.color == LEFT_CONE_COLOUR]
        right_cones = [c for c in cone_pos_msg.cones if c.color == RIGHT_CONE_COLOUR]

        if len(left_cones) == 0 or len(right_cones) == 0:  # no cones
            return

        # order cones by distance from car
        left_cones.sort(key=lambda c: c.location.x)
        right_cones.sort(key=lambda c: c.location.x)

        # make one array with alternating left and right cones
        cones = left_cones + right_cones
        self.track = np.array([[c.location.x, c.location.y] for c in cones])

    def callback(self, msg: PoseWithCovarianceStamped):
        # Only start once the path and map has been recieved,
        # it's a following lap, and we are ready to drive
        if not self.following or not self.r2d or self.path.size == 0:  # or self.track.size == 0:
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
        position: List[float] = self.get_wheel_position(position_cog, theta)

        # rvwp control
        rvwp: List[float] = self.get_rvwp(position)

        des_heading_ang = angle(position, [rvwp[0], rvwp[1]])
        error = wrap_to_pi(theta - des_heading_ang)
        steering_angle = np.rad2deg(error) * self.Kp_ang

        if self.DEBUG_IMG:
            debug_img = self.draw_rvwp(self.path, rvwp, position, steering_angle, self.vel_max)
            self.debug_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))

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
    node = ParticlePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()