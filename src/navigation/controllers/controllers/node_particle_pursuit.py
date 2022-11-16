
import numpy as np

import message_filters
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDrive
from driverless_msgs.msg import PathStamped, Reset
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped


class ParticlePursuit(Node):
    path = np.array([])
    Kp_ang: float = 3
    Kp_vel: float = 0.08
    vel_max: float = 4  # m/s
    vel_min: float = 3  # m/s
    throttle_max: float = 0.2
    brake_max: float = 0.12
    r2d: bool = True  # for reset


    def __init__(self):
        super().__init__("particle_pursuit")

        # sub to delaunay target path mapper for the desired vehicle path (as an array)
        self.create_subscription(PathStamped, "/sim/path", self.path_callback, 10)

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
    ):
        # Drive if ready?:
        if not self.r2d:
            return
        
        # Only start once the path has been recieved
        if self.path.size == 0:
            return

        #----------------
        # Determine steering angle
        #----------------
        f_attractive: float
        pos_target: tuple[float, float] # [x,y]
        pos_car: tuple[float, float]    # [x,y]
        k_attractive: float             # attractive force gain

        f_attractive = k_attractive * (pos_target - pos_car)

        f_repulsive: float
        danger_level: float
        


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

        