from math import atan2, cos, dist, pi, sin, sqrt

import numpy as np
import scipy.spatial
from transforms3d.euler import quat2euler

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import Cone, ConeDetectionStamped, PathStamped, Reset
from geometry_msgs.msg import PoseWithCovarianceStamped

from driverless_common.common import angle, fast_dist, wrap_to_pi

from typing import List

LEFT_CONE_COLOUR = Cone.BLUE
RIGHT_CONE_COLOUR = Cone.YELLOW


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

    return [x_axle, y_axle, heading]


def get_distance(pos_target1: List[float], pos_target2: List[float]) -> float:
    """
    Gets the distance between two points of interest.
    * param pos_target1: [x,y] coords of first point of interest
    * param pos_target2: [x,y] coords of second point of interest
    * return: [float] of distance between two points of interest
    """

    distance = dist(pos_target1, pos_target2)
    return distance


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


def angle_between(v1: List[float], v2: List[float]):
    """
    Find the angle between two vectors, both with [0,0] as origin
    * param v1: [x,y] coords of first vector
    * param v2: [x,y] coords of second vector
    * return: angle in rads between two vectors
    """
    unit_vector_1 = v1 / np.linalg.norm(v1)
    unit_vector_2 = v2 / np.linalg.norm(v2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    angle = np.arccos(dot_product)
    return angle


class ParticlePursuit(Node):
    """
    ADAPTED FROM: https://link.springer.com/chapter/10.1007/978-3-031-10047-5_5
    * Treats the vehicle as a charged point particle, interacting \
    * with two external charged forces (barrier - repulsive, lookahead - attractive).
    """

    # ------------------------------
    # common constants:
    path = np.array([])
    cone_pos = []
    Kp_ang: float = 1.2
    Kp_vel: float = 0.08
    target_vel: float = 3.0
    vel_max: float = 4  # m/s
    vel_min: float = 3  # m/s
    throttle_max: float = 0.2
    brake_max: float = 0.12
    Kp_brake: float = 0.0
    vel_RVWP_LAD: int = 15
    r2d: bool = True  # for reset
    fallback_path_points_offset = 0

    # ------------------------------
    # attractive force constants:
    k_attractive: float = 3  # attractive force gain

    # ------------------------------
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

        # sub to track midline path mapper for the desired vehicle path (as an array), used for lookahead
        self.create_subscription(PathStamped, "/planner/path", self.path_callback, 10)

        # sub to track for all cone locations relative to car start point, used for boundary danger calculations
        self.create_subscription(ConeDetectionStamped, "/planner/interpolated_map", self.track_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/slam/car_pose", self.callback, 10)

        # sync subscribers pose + velocity
        # pose_sub = message_filters.Subscriber(self, PoseWithCovarianceStamped, "/slam/car_pose")
        # vel_sub = message_filters.Subscriber(self, TwistWithCovarianceStamped, "/imu/velocity")
        # synchronizer = message_filters.ApproximateTimeSynchronizer(fs=[pose_sub, vel_sub], queue_size=20, slop=0.2)
        # synchronizer.registerCallback(self.callback)

        self.reset_sub = self.create_subscription(Reset, "/reset", self.reset_callback, 10)

        # publishers
        self.control_publisher: Publisher = self.create_publisher(AckermannDriveStamped, "/control/driving_command", 10)

        # lookahead distance for the RVWP in meters
        self.lookahead = self.declare_parameter("lookahead", 6.5).value

        self.get_logger().info("---Path Follower Node Initalised---")
        print("The node is initialised!")

    # AS start button check?
    def reset_callback(self, reset_msg: Reset):
        self.path = np.array([])
        self.r2d = True
        # print("Ready to drive!")

    # 'recieve' the path
    def path_callback(self, spline_path_msg: PathStamped):
        # convert List[PathPoint] to 2D numpy array
        if self.path.shape != (0,):
            return
        self.path = np.array([[p.location.x, p.location.y, p.turn_intensity] for p in spline_path_msg.path])
        self.get_logger().info(f"Spline Path Recieved - length: {len(self.path)}")

    # recieve the cone locations
    def track_callback(self, cone_pos_msg: ConeDetectionStamped):
        self.cone_pos = cone_pos_msg.cones
        self.get_logger().debug("Map received")

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
            # ensure distance is greater than lookahead
            distance = fast_dist(p, close)
            if distance <= self.lookahead**2 or distance >= rvwp_dist:
                continue

            # get angle to check if the point is in front of the car
            ang = wrap_to_pi(angle(car_pos[:2], p))

            # normalize the car's orientation angle
            car_theta = wrap_to_pi(car_pos[2])

            # calculate the angle error
            error = wrap_to_pi(car_theta - ang)

            if (
                2 * np.pi / 3 > error > -2 * np.pi / 3
            ):  # Checking if the point is in a 240 degree range infront of the vehicle
                rvwp_dist = distance
                rvwp_index = i

            # # get angle to check if the point is in front of the car
            # ang = angle(car_pos[:2], p)
            # error = wrap_to_pi(car_pos[2] - ang)
            # if (
            #     2 * np.pi / 3 > error > -2 * np.pi / 3
            # ):  # Checking if the point is in a 240 degree range infront of the vehicle
            #     rvwp_dist = distance
            #     rvwp_index = i
            #     # print("CHOSEN HAS DISTANCE: " + str(sqrt(distance)))

        if rvwp_index is None or rvwp_index == close_index:
            self.get_logger().warn("No valid RVWP found, using fallback point")
            path_points_count = len(self.path) - 1
            fallback_point = close_index + self.fallback_path_points_offset
            if fallback_point > path_points_count:
                rvwp_index = abs(path_points_count - fallback_point)
            else:
                rvwp_index = fallback_point

        pos_lookahead = self.path[rvwp_index]

        RVWP_distance = get_distance(car_pos[:2], pos_lookahead[:2])
        print(
            "RVWP Index: "
            + str(rvwp_index)
            + "\nRVWP Pos: "
            + str(pos_lookahead[:2])
            + "\nRVWP Dist: "
            + str(RVWP_distance)
        )
        return pos_lookahead

    def callback(
        self,
        pose_msg: PoseWithCovarianceStamped,
    ):
        # ----------------
        # Node initialisation processes:
        # ----------------
        # Drive if ready:
        if not self.r2d:
            return

        # Only start if the path has been recieved:
        if self.path.size == 0:
            return

        # ----------------
        # Get positions of car and cones
        # ----------------
        # i, j, k angles in rad
        car_heading = quat2euler(
            [
                pose_msg.pose.pose.orientation.w,
                pose_msg.pose.pose.orientation.x,
                pose_msg.pose.pose.orientation.y,
                pose_msg.pose.pose.orientation.z,
            ]
        )[2]

        # get the position of the centre of the front steering axle
        position_cog: List[float] = [pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y]
        car_pose: List[float] = get_wheel_position(position_cog, car_heading)  # [x,y,th] of centre of steering axle

        # get left and right cones
        left_cones = [c for c in self.cone_pos if c.color == LEFT_CONE_COLOUR]
        right_cones = [c for c in self.cone_pos if c.color == RIGHT_CONE_COLOUR]

        if len(left_cones) == 0 or len(right_cones) == 0:  # no cones
            return

        # order cones by distance from car
        left_cones.sort(key=lambda c: c.location.x)
        right_cones.sort(key=lambda c: c.location.x)

        # make one array with alternating left and right cones
        cones = left_cones + right_cones
        track = np.array([[c.location.x, c.location.y] for c in cones])

        # ----------------
        # Determine steering angle
        # ----------------
        pos_lookahead: List[float] = self.get_rvwp(car_pose)

        pos_nearestCone: List[float] = get_closest_cone(car_pose, track)
        d_nearestCone: float = get_distance(car_pose[:2], pos_nearestCone)

        # danger_level is a scalar of 0-1 for f_repulsive, determined by distance to nearest cone
        danger_level: float = np.clip(
            (d_nearestCone ** (1 - self.cone_danger) - self.d_max ** (1 - self.cone_danger))
            / (self.d_min ** (1 - self.cone_danger) - self.d_max ** (1 - self.cone_danger)),
            0,
            1,
        )

        # determine attractive and repulsive forces acting on car
        f_attractive: float = self.k_attractive * (
            get_distance(pos_lookahead[:2], car_pose[:2])
        )  # car is attracted to lookahead
        f_repulsive: float = self.k_repulsive * danger_level  # car is repulsed by nearest cone

        # determine angles of forces acting on car (angles in rads)
        attractive_heading: float = angle(car_pose[:2], pos_lookahead[:2])
        repulsive_heading: float = angle(car_pose[:2], pos_nearestCone) + pi  # opposite heading of position

        # set repulsive heading perpendicular to current car heading (away from pos_nearestCone)
        if repulsive_heading > car_heading and repulsive_heading <= car_heading + pi:
            repulsive_heading = car_heading + 0.5 * pi
        else:
            repulsive_heading = car_heading - 0.5 * pi

        # get coords of vectors (forces) acting on car, relative to the car as origin
        pos_attractive_relCar: List[float] = [
            f_attractive * cos(attractive_heading),
            f_attractive * sin(attractive_heading),
        ]
        pos_repulsive_relcar: List[float] = [f_repulsive * cos(repulsive_heading), f_repulsive * sin(repulsive_heading)]

        # get coords of resultant vector (force) acting on car, relative to the car as origin
        pos_resultant_relCar: List[float] = [
            pos_attractive_relCar[0] + pos_repulsive_relcar[0],
            pos_attractive_relCar[1] + pos_repulsive_relcar[1],
        ]

        # get angle of resultant vector as desired heading of the car in degrees
        des_heading_ang: float = (angle([0, 0], pos_resultant_relCar)) * 180 / pi
        steering_angle: float = ((car_heading * 180 / pi) - des_heading_ang) * self.Kp_ang * -1

        # ----------------
        # Determine velocity
        # ----------------

        target_vel: float = self.target_vel

        """
        This Section is currently uneccessary (also ideally should be located in a velocity planner package).
        This code would be used when the velocity of the vehicle would be reaching the limit of friction on the tires,
        and thus this code would determine what the maximum safe velocities would be.
        """

        # velocity control
        # rvwp: List[float] = get_RVWP(pos_car, self.path, self.vel_RVWP_LAD)
        # intensity = rvwp[2]
        # vel = sqrt(vel_msg.twist.twist.linear.x**2 + vel_msg.twist.twist.linear.y**2)

        # target velocity proportional to angle
        # target_vel: float = (
        #     self.vel_max - intensity * self.Kp_vel
        # )

        # if target_vel < self.vel_min:
        #     target_vel = self.vel_min

        # # increase proportionally as it approaches target
        # throttle_scalar: float = 1 - (vel / target_vel)
        # calc_brake = 0.0
        # if throttle_scalar > 0:
        #     calc_throttle = self.throttle_max * throttle_scalar
        # # if its over maximum, brake propotionally unless under minimum
        # else:
        #     calc_throttle = 0.0
        #     if vel > self.vel_min:
        #         calc_brake = abs(self.brake_max * throttle_scalar) * intensity * self.Kp_brake

        # ----------------
        # publish message
        # ----------------
        control_msg = AckermannDriveStamped()
        control_msg.drive.steering_angle = steering_angle
        control_msg.drive.speed = target_vel
        # control_msg.drive.jerk = calc_brake  # jerk redundant with new sim

        self.control_publisher.publish(control_msg)

        # ======================================================
        #                   Debugging Code
        # ======================================================

        # Graphing of cones:
        # import matplotlib.pyplot as plt

        # # Separate the x and y coordinates
        # x = [cone.location.x for cone in self.cone_pos]
        # y = [cone.location.y for cone in self.cone_pos]

        # # Plot the points
        # plt.scatter(x, y)

        # # Add labels and a title
        # plt.xlabel("x - axis")
        # plt.ylabel("y - axis")
        # plt.title("2D Plane Plot of Ordered Cones")

        # # Display the plot
        # plt.show()


def main(args=None):  # begin ros node
    rclpy.init(args=args)
    node = ParticlePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
