from math import cos, dist, pi, sin

import cv2
import numpy as np

from cv_bridge import CvBridge
import rclpy

from driverless_msgs.msg import ConeDetectionStamped

from driverless_common.common import angle, fast_dist, wrap_to_pi
from pure_pursuit.node_pure_pursuit import PurePursuit

from typing import List

cv_bridge = CvBridge()


class ParticlePursuit(PurePursuit):
    """
    This inherits from PurePursuit as the underlying path following algorithm is the same

    Avoidance adapted from: https://link.springer.com/chapter/10.1007/978-3-031-10047-5_5
    * Treats the vehicle as a charged point particle, interacting \
    * with two external charged forces (barrier - repulsive, lookahead - attractive).
    """

    # ------------------------------
    track = None

    # attractive force constants:
    k_attractive: float = 1  # attractive force gain

    # repulsive force constants:
    d_min: float = 1.5  # min repulsive force distance (max. repulsion at or below)
    d_max: float = 2.0  # max repulsive force distance (zero repulsion at or above)
    k_repulsive: float = 0.1  # repulsive force gain

    # cone_danger - a unitless, *inverse* 'spring constant' of the repulsive force (gamma in documentation)
    # E.g. cone_danger > 0: corners cut tighter
    #      cone_danger < 0: corners taken wider
    #      ** dont set to 1.0 **
    cone_danger: float = 0.0

    last_pos_nearest_cone = [0, 0]
    last_pos_attractive_rel_car = [0, 0]
    last_pos_repulsive_rel_car = [0, 0]
    last_pos_resultant_rel_car = [0, 0]

    def __init__(self):
        super().__init__(node_name="particle_pursuit_node")

        # sub to track for all cone locations relative to car start point, used for boundary danger calculations
        self.create_subscription(ConeDetectionStamped, "/planner/interpolated_map", self.interp_track_callback, 10)

        self.get_logger().info("---Particle pursuit follower initalised---")

    # recieve the cone locations
    def interp_track_callback(self, cone_pos_msg: ConeDetectionStamped):
        self.get_logger().debug("Map received")

        self.track = np.array([[c.location.x, c.location.y] for c in cone_pos_msg.cones])

    def can_drive(self):
        """
        Overrides the can_drive method in PurePursuit.
        """

        if self.path is None or self.track is None:
            return False
        self.get_logger().info("Can drive", once=True)
        return True

    def get_closest_cone(self, pos_car: List[float]) -> List[float]:
        """
        Gets the position of the nearest cone to the car.
        * param pos_car: [x,y] coords of car position
        * param boundaries: [x,y] coords of all current cones
        * return: [x,y] of nearest cone to the car
        """
        # iterate through all cones and find the closest one
        nearest_cone = None
        last_dist = float("inf")
        for cone in self.track:
            current_dist = fast_dist(pos_car, cone)
            if current_dist < last_dist:
                nearest_cone = cone
                last_dist = current_dist

        return nearest_cone

    def calc_steering(self, pose: List[float], rvwp: List[float]) -> float:
        """
        Overrides the calc_steering method in PurePursuit.
        Calculates the steering angle to follow the path using a potential field approach.
        Here, the vehicle is treated as a charged point particle, interacting with two external charged forces.
        - closest boundary point: repulsive
        - lookahead rvwp: attractive
        """

        # avoidance from potential field
        # get position of nearest cone
        pos_nearest_cone = self.get_closest_cone(pose[:2])
        if pos_nearest_cone is None:
            return 0  # no cones detected, dont steer

        # maybe use fast_dist instead of dist later
        d_nearestCone = dist(pose[:2], pos_nearest_cone)

        # danger_level is a scalar of 0-1 for f_repulsive, determined by distance to nearest cone
        danger_level: float = np.clip(
            (d_nearestCone ** (1 - self.cone_danger) - self.d_max ** (1 - self.cone_danger))
            / (self.d_min ** (1 - self.cone_danger) - self.d_max ** (1 - self.cone_danger)),
            0,
            1,
        )

        # determine attractive and repulsive forces acting on car
        f_attractive: float = self.k_attractive * (dist(rvwp[:2], pose[:2]))  # car is attracted to lookahead
        f_repulsive: float = self.k_repulsive * danger_level  # car is repulsed by nearest cone

        # determine angles of forces acting on car (angles in rads)
        attractive_heading: float = wrap_to_pi(angle(pose[:2], rvwp[:2]))
        repulsive_heading: float = wrap_to_pi(angle(pose[:2], pos_nearest_cone) + pi)  # opposite heading of position

        # angle from car's current heading to nearest cone
        ang_vehicle_to_cone = wrap_to_pi(angle(pose[:2], pos_nearest_cone) - pose[2])

        # set repulsive heading perpendicular to current car heading (away from pos_nearestCone)
        if ang_vehicle_to_cone > 0:
            repulsive_heading = wrap_to_pi(pose[2] + 0.5 * pi)
        else:
            repulsive_heading = wrap_to_pi(pose[2] - 0.5 * pi)

        # get coords of vectors (forces) acting on car, relative to the car as origin
        pos_attractive_rel_car: List[float] = [
            f_attractive * cos(attractive_heading),
            f_attractive * sin(attractive_heading),
        ]
        pos_repulsive_rel_car: List[float] = [
            -f_repulsive * cos(repulsive_heading),
            -f_repulsive * sin(repulsive_heading),
        ]

        # get coords of resultant vector (force) acting on car, relative to the car as origin
        pos_resultant_rel_car: List[float] = [
            pos_attractive_rel_car[0] + pos_repulsive_rel_car[0],
            pos_attractive_rel_car[1] + pos_repulsive_rel_car[1],
        ]

        self.last_pos_attractive_rel_car = pos_attractive_rel_car
        self.last_pos_repulsive_rel_car = pos_repulsive_rel_car
        self.last_pos_resultant_rel_car = pos_resultant_rel_car
        self.last_pos_nearest_cone = pos_nearest_cone

        # get angle of resultant vector as desired heading of the car in degrees
        des_heading_ang = angle([0, 0], pos_resultant_rel_car)
        error = wrap_to_pi(pose[2] - des_heading_ang)
        steering = np.rad2deg(error) * self.Kp_ang
        return steering

    def draw_forces(self, debug_img, img_params, position):
        scale = img_params[0]
        x_offset = img_params[1]
        y_offset = img_params[2]

        # draw the nearest cone
        cv2.drawMarker(
            debug_img,
            (
                int(self.last_pos_nearest_cone[0] * scale + x_offset),
                int(self.last_pos_nearest_cone[1] * scale + y_offset),
            ),
            (255, 255, 0),
            markerType=cv2.MARKER_TRIANGLE_UP,
            markerSize=10,
            thickness=4,
        )
        # draw the attractive force
        cv2.arrowedLine(
            debug_img,
            (int(position[0] * scale + x_offset), int(position[1] * scale + y_offset)),
            (
                int((position[0] + self.last_pos_attractive_rel_car[0]) * scale + x_offset),
                int((position[1] + self.last_pos_attractive_rel_car[1]) * scale + y_offset),
            ),
            (0, 255, 0),
            3,
        )
        # draw the repulsive force
        cv2.arrowedLine(
            debug_img,
            (int(position[0] * scale + x_offset), int(position[1] * scale + y_offset)),
            (
                int((position[0] + self.last_pos_repulsive_rel_car[0]) * scale + x_offset),
                int((position[1] + self.last_pos_repulsive_rel_car[1]) * scale + y_offset),
            ),
            (0, 0, 255),
            3,
        )

        return debug_img

    def publish_debug_image(self, steering_angle: float, velocity: float, rvwp: List[float], position: List[float]):
        """
        Overrides the publish_debug_image method in PurePursuit.
        Adds draw_forces as a step in the debug image generation.
        """

        img_params = self.get_img_params()
        debug_img = self.draw_rvwp(img_params, rvwp, position)
        debug_img = self.draw_forces(debug_img, img_params, position)
        debug_img = self.add_data_text(debug_img, steering_angle, velocity)

        self.debug_pub.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))


def main(args=None):  # begin ros node
    rclpy.init(args=args)
    node = ParticlePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
