from math import cos, sin, dist, atan2, pi, sqrt

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
    dists: np.ndarray = scipy.spatial.distance.cdist(boundaries_coords, _pos, "euclidean",)
    # not certain np.where is returning the index - it should though
    nearest_cone_index: int = np.where(dists == np.amin(dists))[0][0]    
      
    nearest_cone = boundaries_coords[nearest_cone_index]
    return nearest_cone

def get_RVWP(car_pos: List[float], path: np.ndarray, rvwp_lookahead: int) -> List[float]:
    """
    Retrieve position of lookahead target
    * param car_pos: [x,y] coords of point 1
    * param path: [[x0,y0],[x1,y1],...,[xn-1,yn-1]] path points
    * param rvwpLookahead: how many indices to look ahead in path array for RVWP
    * return: RVWP position as [x,y,intensity]
    """
    _pos = np.array([[car_pos[0], car_pos[1]]])
    dists: np.ndarray = scipy.spatial.distance.cdist(
        path[:, :2],  # search all path points for x,y cols up to 3rd col (intensity)
        _pos,
        "euclidean",
    )
    min_index: int = np.where(dists == np.amin(dists))[0][0]
    #print("min_index: ", min_index)
    if min_index + rvwp_lookahead >= len(path):
        rvwp_index: int = len(path) - 1
    else:
        rvwp_index: int = min_index + rvwp_lookahead  # % len(path)
    #print("rvwp_index: ", rvwp_index)
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
    return (angle + np.pi) % (2 * np.pi) - np.pi

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
    return(angle)

class ParticlePursuit(Node):
    """
    ADAPTED FROM: https://link.springer.com/chapter/10.1007/978-3-031-10047-5_5
    * Treats the vehicle as a charged point particle, interacting \
    * with two external charged forces (barrier - repulsive, lookahead - attractive).
    """
    #------------------------------
    # common constants:
    path = np.array([])
    cone_pos = []
    Kp_ang: float = 3
    Kp_vel: float = 0.08
    vel_max: float = 4  # m/s
    vel_min: float = 3  # m/s
    throttle_max: float = 0.2
    brake_max: float = 0.12
    Kp_brake: float = 0.0
    vel_RVWP_LAD: int = 15
    r2d: bool = True  # for reset
    
    #------------------------------
    # attractive force constants:
    rvwp_lookahead: float = 100  # how far the lookahead is (no. of indeces) [convert to distance preferably]
    k_attractive: float = 3     # attractive force gain

    #------------------------------
    # repulsive force constants:
    d_min: float = 1.3             # min repulsive force distance (max. repulsion at or below)    
    d_max: float = 2.0           # max repulsive force distance (zero repulsion at or above)     
    k_repulsive: float = 10      # repulsive force gain
    
    # cone_danger - a unitless, *inverse* 'spring constant' of the repulsive force (gamma in documentation)
    # E.g. cone_danger > 0: corners cut tighter
    #      cone_danger < 0: corners taken wider
    #      ** dont set to 1.0 **
    cone_danger: float = 18     
    

    def __init__(self):
        super().__init__("particle_pursuit")

        # sub to delaunay target path mapper for the desired vehicle path (as an array), used for lookahead
        self.create_subscription(PathStamped, "/sim/path", self.path_callback, 10)

        # sub to track for all cone locations relative to car start point, used for boundary danger calculations
        self.create_subscription(TrackDetectionStamped, "/sim/track", self.track_callback, 10)

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

    # recieve the cone locations
    def track_callback(self, cone_pos_msg: TrackDetectionStamped):
        self.cone_pos = cone_pos_msg.cones

    def callback(
        self,
        pose_msg: PoseWithCovarianceStamped,
        vel_msg: TwistWithCovarianceStamped,
    ):
        #----------------
        # Node initialisation processes:
        #----------------
        # Drive if ready:
        if not self.r2d:
            return
        
        # Only start if the path has been recieved:
        if self.path.size == 0:
            return

        #----------------
        # Get positions of car and cones
        #----------------
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
        pos_car: List[float] = get_wheel_position(position_cog, car_heading) #[x,y] of centre of steering axle

        # get left and right cones
        left_cones = [c.cone for c in self.cone_pos if c.cone.color == LEFT_CONE_COLOUR]
        right_cones = [c.cone for c in self.cone_pos if c.cone.color == RIGHT_CONE_COLOUR]

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
        pos_lookahead: List[float] = get_RVWP(pos_car, self.path, self.rvwp_lookahead)[:2]  

        pos_nearestCone: List[float] = get_closest_cone(pos_car, track)
        d_nearestCone: float = get_distance(pos_car, pos_nearestCone)

        # danger_level is a scalar of 0-1 for f_repulsive, determined by distance to nearest cone
        danger_level: float = np.clip((d_nearestCone**(1-self.cone_danger) - self.d_max**(1-self.cone_danger)) \
                                       / (self.d_min**(1-self.cone_danger) - self.d_max**(1-self.cone_danger)), 0, 1)

        # determine attractive and repulsive forces acting on car
        f_attractive: float = self.k_attractive * (get_distance(pos_lookahead, pos_car))        # car is attracted to lookahead
        f_repulsive: float = self.k_repulsive * danger_level                                    # car is repulsed by nearest cone
        
        # determine angles of forces acting on car (angles in rads)
        attractive_heading: float = angle(pos_car, pos_lookahead)
        repulsive_heading: float = angle(pos_car, pos_nearestCone) + pi                        # opposite heading of position

        # set repulsive heading perpendicular to current car heading (away from pos_nearestCone)
        if repulsive_heading > car_heading and repulsive_heading <= car_heading + pi:
            repulsive_heading = car_heading + 0.5 * pi
        else:
            repulsive_heading = car_heading - 0.5 * pi

        # get coords of vectors (forces) acting on car, relative to the car as origin
        pos_attractive_relCar: List[float] = [f_attractive * cos(attractive_heading), f_attractive * sin(attractive_heading)]
        pos_repulsive_relcar: List[float] = [f_repulsive * cos(repulsive_heading), f_repulsive * sin(repulsive_heading)]

        # get coords of resultant vector (force) acting on car, relative to the car as origin
        pos_resultant_relCar: List[float] = [pos_attractive_relCar[0] + pos_repulsive_relcar[0], pos_attractive_relCar[1] + pos_repulsive_relcar[1]]

        # get angle of resultant vector as desired heading of the car
        des_heading_ang: float = angle([0,0], pos_resultant_relCar)
        steering_angle = wrap_to_pi(car_heading - des_heading_ang) * self.Kp_ang

        print("d_nearestCone: " + str(d_nearestCone))
        print("danger_level: " + str(danger_level))

        #----------------
        # Determine velocity
        #----------------
        # velocity control
        rvwp: List[float] = get_RVWP(pos_car, self.path, self.vel_RVWP_LAD)
        intensity = rvwp[2]
        vel = sqrt(vel_msg.twist.twist.linear.x**2 + vel_msg.twist.twist.linear.y**2)

        # target velocity proportional to angle
        target_vel: float = self.vel_max - intensity * self.Kp_vel
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
                calc_brake = abs(self.brake_max * throttle_scalar) * intensity * self.Kp_brake
        
        #----------------
        # publish message
        #----------------
        control_msg = AckermannDrive()
        control_msg.steering_angle = steering_angle
        control_msg.acceleration = calc_throttle
        control_msg.jerk = calc_brake  # using jerk for brake for now

        self.control_publisher.publish(control_msg)
        
        
def main(args=None):  # begin ros node
    rclpy.init(args=args)
    node = ParticlePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()