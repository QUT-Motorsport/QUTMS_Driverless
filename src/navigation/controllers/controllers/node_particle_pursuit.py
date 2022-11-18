from math import cos, sin, sqrt

import numpy as np
import scipy.spatial
from transforms3d.euler import quat2euler

import message_filters
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDrive
from driverless_msgs.msg import PathStamped, Reset, TrackDetectionStamped, Cone, ConeWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped

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

    return [x_axle, y_axle]

def get_distance(pos_target1: tuple[float, float], pos_target2: tuple[float, float]) -> float:
    """
    Gets the distance between two points of interest.
    * param pos_target1: [x,y] coords of first point of interest
    * param pos_target2: [x,y] coords of second point of interest
    * return: [float] of distance between two points of interest
    """

    distance = scipy.spatial.distance.cdist(pos_target1, pos_target2, "euclidean")
    return distance

def get_closest_cone(pos_car: tuple[float, float], boundaries: tuple[:, :2]) -> float:
    """
    Gets the position of the nearest cone to the car.
    * param pos_car: [x,y] coords of car position
    * param boundaries: [x,y,~] coords of all current cones
    * return: [x,y] of nearest cone to the car
    """
    
    _pos = np.array([[pos_car[0], pos_car[1]]])
    dists: np.ndarray = scipy.spatial.distance.cdist(
        boundaries[:, :2],  # search all path points for x,y cols up to 3rd col (intensity)
        _pos,
        "euclidean",
    )
    nearest_cone_distance: tuple[float, float] = np.where(dists == np.amin(dists))[0][0]

    # find the index of the cone with the lowest distance to the car
    for cone in dists:
        if dists[cone] == nearest_cone_distance:
            nearest_cone_index = cone
            
    nearest_cone = boundaries[nearest_cone_index]
    return nearest_cone

def get_RVWP(car_pos: List[float], path: np.ndarray, rvwp_lookahead: int) -> List[float]:
    """
    Retrieve position of lookahead target
    * param car_pos: [x,y] coords of point 1
    * param path: [[x0,y0],[x1,y1],...,[xn-1,yn-1]] path points
    * param rvwpLookahead: how many indices to look ahead in path array for RVWP
    * return: RVWP position as [x,y]
    """
    _pos = np.array([[car_pos[0], car_pos[1]]])
    dists: np.ndarray = scipy.spatial.distance.cdist(
        path[:, :2],  # search all path points for x,y cols up to 3rd col (intensity)
        _pos,
        "euclidean",
    )
    min_index: int = np.where(dists == np.amin(dists))[0][0]
    print("min_index: ", min_index)
    if min_index + rvwp_lookahead >= len(path):
        rvwp_index: int = len(path) - 1
    else:
        rvwp_index: int = min_index + rvwp_lookahead  # % len(path)
    print("rvwp_index: ", rvwp_index)
    rvwp: List[float] = path[rvwp_index]

    return rvwp

class ParticlePursuit(Node):
    """
    ADAPTED FROM: https://link.springer.com/chapter/10.1007/978-3-031-10047-5_5
    Treats the vehicle as a charged point particle, interacting
    with two external charged forces (barrier - repulsive, lookahead - attractive).
    """
    
    # basic constants
    path = np.array([])
    Kp_ang: float = 3
    Kp_vel: float = 0.08
    vel_max: float = 4  # m/s
    vel_min: float = 3  # m/s
    throttle_max: float = 0.2
    brake_max: float = 0.12
    r2d: bool = True  # for reset
    
    # attractive force constants
    rvwp_lookahead: float = 15  # how far the lookahead is (no. of indeces) [convert to distance preferably]
    k_attractive: float = 1     # attractive force gain

    # repulsive force constants
    d_min: float = 1.2          # min repulsive force distance (car cant go closer to cone)
    d_max: float = 2.8          # max repulsive force distance
    k_repulsive: float = 1      # repulsive force gain
    boundary_danger: float = 0.75      # danger_level gain (sort of), gamma in documentation
    


    def __init__(self):
        super().__init__("particle_pursuit")

        # sub to delaunay target path mapper for the desired vehicle path (as an array), used for lookahead
        self.create_subscription(PathStamped, "/sim/path", self.path_callback, 10)

        # sub to track for all cone locations relative to car start point, used for boundary danger calculations
        self.create_subscription(TrackDetectionStamped, "/slam/track", self.callback, 10)

        # sync subscribers pose + velocity
        pose_sub = message_filters.Subscriber(self, PoseWithCovarianceStamped, "/zed2i/zed_node/pose_with_covariance")
        vel_sub = message_filters.Subscriber(self, TwistWithCovarianceStamped, "/imu/velocity")
        synchronizer = message_filters.ApproximateTimeSynchronizer(fs=[pose_sub, vel_sub], queue_size=20, slop=0.2)
        synchronizer.registerCallback(self.callback)

        self.reset_sub = self.create_subscription(Reset, "/reset", self.reset_callback, 10)

        # publishers
        self.control_publisher: Publisher = self.create_publisher(AckermannDrive, "/driving_command", 10)

        self.get_logger().info("---Path Follower Node Initalised---")

    # AS start button check?
    def reset_callback(self, reset_msg: Reset):
        self.path = np.array([])
        self.r2d = True

    # 'recieve' the path
    def path_callback(self, spline_path_msg: PathStamped):
        # convert List[PathPoint] to 2D numpy array
        self.path = np.array([[p.location.x, p.location.y, p.turn_intensity] for p in spline_path_msg.path])
        self.get_logger().debug(f"Spline Path Recieved - length: {len(self.path)}")

    def callback(
        self,
        pose_msg: PoseWithCovarianceStamped,
        vel_msg: TwistWithCovarianceStamped,
        track_msg: TrackDetectionStamped,
    ):
        #----------------
        # Node initialisation processes:
        #----------------
        # Drive if ready:
        if not self.r2d:
            return
        
        # Only start once the path has been recieved
        if self.path.size == 0:
            return

        #----------------
        # Get positions of car and cones
        #----------------
        # i, j, k angles in rad
        theta = quat2euler(
            [
                pose_msg.pose.pose.orientation.w,
                pose_msg.pose.pose.orientation.x,
                pose_msg.pose.pose.orientation.y,
                pose_msg.pose.pose.orientation.z,
            ]
        )[2]

        # get the position of the centre of the front steering axle
        position_cog: List[float] = [pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y]
        position: List[float] = get_wheel_position(position_cog, theta) #[x,y] of centre of steering axle

        cones_with_cov: List[ConeWithCovariance] = track_msg.cones

        # get left and right cones
        left_cones = [c.cone for c in cones_with_cov if c.cone.color == LEFT_CONE_COLOUR]
        right_cones = [c.cone for c in cones_with_cov if c.cone.color == RIGHT_CONE_COLOUR]

        if len(left_cones) == 0 or len(right_cones) == 0:  # no cones
            return

        # order cones by distance from car
        left_cones.sort(key=lambda c: c.location.x)
        right_cones.sort(key=lambda c: c.location.x)

        # make one array with alternating left and right cones
        cones = left_cones + right_cones
        track = np.array([[c.location.x, c.location.y] for c in cones])

        #----------------
        # Determine steering angle
        #----------------
        pos_car: tuple[float, float]  = position    # [x,y]
        pos_lookahead: tuple[float, float] = get_RVWP(pos_car, self.path, self.rvwp_lookahead)  

        f_attractive = self.k_attractive * (get_distance(pos_lookahead, pos_car))

        pos_nearestBoundary: tuple[float, float] = get_closest_cone(pos_car, track)
        d_nearestBoundary: float = get_distance(pos_car, pos_nearestBoundary)

        # danger_level = [max] [lim(gamma approaching gamma(k)?) of] [(d_nearestBoundary(x,y) - d_max) / (d_min - d_max)]
        # need to take the maximum value of the fraction as it approaches the limit of gamma(k) which 
        # i "think" is just another way of saying as x approaches gamma, gamma(k) being the 
        # final value processed?
        danger_level_expression = (d_nearestBoundary**(1-self.boundary_danger) - self.d_max**(1-self.boundary_danger)) \
                                 / (self.d_min**(1-self.boundary_danger) - self.d_max**(1-self.boundary_danger))
        
        danger_level = np.amax(             )

        # f_repulsive = self.k_repulsive * danger_level * ((p - p_danger) / abs(p - p_danger))
        f_repulsive: float
        
        #----------------
        # Determine acceleration (integrate velocity)
        #----------------


        # temp initialisation
        steering_angle: float
        calc_throttle: float
        calc_brake: float

        # publish message
        control_msg = AckermannDrive()
        control_msg.steering_angle = steering_angle
        control_msg.acceleration = calc_throttle
        control_msg.jerk = calc_brake  # using jerk for brake for now

        self.control_publisher.publish(control_msg)

        