# ROS2 libraries
import rclpy
from rclpy.node import Node
# ROS2 message libraries
from sensor_msgs.msg import PointCloud2, Imu, NavSatFix
from geometry_msgs.msg import TwistStamped
# custom sim data message libraries
from fs_msgs.msg import ControlCommand
from qutms_msgs.msg import ConeData, Location
# helper math function
from .sub_module.simple_lidar import find_points
import math

class Controller(Node):
    def __init__(self):
        super().__init__('control')
        # ## creates subscriber to 'imu' with type Imu that sends data to imu_callback
        # self.imu_subscription_ = self.create_subscription(
        #     Imu,
        #     '/fsds/imu',
        #     self.imu_callback,
        #     10)
        # self.imu_subscription_ 

        # ## creates subscriber to 'gps' with type NavSatFix that sends data to gps_callback
        # self.gps_subscription_ = self.create_subscription(
        #     NavSatFix,
        #     '/fsds/gps',
        #     self.gps_callback,
        #     10)
        # self.gps_subscription_ 

        ## creates subscriber to 'Lidar2' with type PointCloud2 that sends data to lidar_callback
        self.lidar_subscription_ = self.create_subscription(
            PointCloud2,
            '/fsds/lidar/Lidar2',
            self.lidar_callback,
            10)
        self.lidar_subscription_  # prevent unused variable warning

        ## creates subscriber to 'gss' with type TwistStamped that sends data to geo_callback
        self.imu_subscription_ = self.create_subscription(
            TwistStamped,
            '/fsds/gss',
            self.geo_callback,
            10)
        self.geo_subscription_ 

        ## creates publisher to 'control_command' with type ControlCommand
        self.movement_publisher_ = self.create_publisher(
            ControlCommand,
            '/fsds/control_command', 
            10)
        # creates timer for publishing commands
        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.pub_callback)

        ## initial class variables
        self.time = 0
        self.close_cones = []
        self.far_cones = []
        self.sum_far_avg_y = 0 # integral components
        self.sum_close_avg_y = 0 
        self.prev_close_avg_y = 0 # derivative components
        self.prev_far_avg_y = 0
        self.vel = 0.0

        ## test demo variables to change

        ## MODIFY THESE AND MAKE COPIES OF WORKING CONFIGURATIONS
        ## DEMO 2.5 CONSTANTS
        # cone point vars
        self.close_range_cutoff = 6.5 # m
        self.far_range_cutoff = 11 # m
        self.distance_cutoff = 0.1 # m

        # PID coeffs
        self.steering_pc = 0.057
        self.steering_ic = 0
        self.steering_dc = 0.85
        self.steering_pf = 0.009
        self.steering_if = 0
        self.steering_df = 0.45

        # accel + vel targets
        self.vel_p = 0.7
        self.max_throttle = 0.20 # m/s^2
        self.max_vel = 6 # m/s
        self.target_vel = self.max_vel # initially target is max
        self.min_vel = self.max_vel / 2
        self.brake_p = 0.1

        ## DEMO 2.4 CONSTANTS
        # cone point vars
        # self.close_range_cutoff = 6.5 # m
        # self.far_range_cutoff = 11 # m
        # self.distance_cutoff = 0.1 # m

        # # PID coeffs
        # self.steering_pc = 0.08
        # self.steering_ic = 0
        # self.steering_dc = 0.8
        # self.diff_p = 0.04

        # # accel + vel targets
        # self.vel_p = 0.7
        # self.max_throttle = 0.2 # m/s^2
        # self.max_vel = 6 # m/s
        # self.target_vel = self.max_vel # initially target is max


    ## callback for lidar data to be sent to. used to call funtion to find cone coords
    def lidar_callback(self, pcl_msg):
        """ In here, we will call calculations to ideally get the 
        distance, angle, and reflectivity of the cones"""
        
        cone_sets = find_points(pcl_msg, self.close_range_cutoff, self.far_range_cutoff, self.distance_cutoff)
        self.close_cones = cone_sets[0]
        self.far_cones = cone_sets[1]
        
    ## callback for geometry data to be sent to. used to find var's current velocity
    def geo_callback(self, geo_msg):
        vel_x = geo_msg.twist.linear.x
        vel_y = geo_msg.twist.linear.y

        self.vel = math.sqrt(vel_x*vel_x + vel_y*vel_y)

    ## helper function to find the average y "centre" of the cones. this is calculated wrt the FOV centre
    def find_avg_y(self, cone_set):
        length = len(cone_set)
        if length != 0:
            average_y = 0
            for cone in cone_set:
                average_y += cone[1]
            average_y = average_y / length

            return -average_y
        
        else:
            return 0 

    ## helper function to adjust velocity based on how tight the turn is ahead
    def predict_vel(self, close_avg_y, far_avg_y):
        cone_diff = abs(far_avg_y - close_avg_y)
        self.target_vel = self.max_vel - cone_diff*abs(cone_diff) * self.vel_p

        calc_brake = 0

        if self.target_vel < self.min_vel:
            vel_diff = self.min_vel - self.target_vel
            calc_brake = vel_diff * self.brake_p
            self.target_vel = self.min_vel # stop it from slowing too much

        return calc_brake


    ## callback for publishing FSDS command messages at specific times
    def pub_callback(self):
        self.time += self.timer_period # increase time taken

        # initialise variables
        calc_throttle = 0.0 
        calc_brake = 0.0
        calc_steering = 0.0

        # call cone detection
        close_avg_y = self.find_avg_y(self.close_cones) # frame of reference is flipped along FOV (for some reason)
        far_avg_y = self.find_avg_y(self.far_cones) # frame of reference is flipped along FOV (for some reason)
        
        # wait for initial publishing delay
        if (self.time >= 3): 
            # calculate throttle + brake
            calc_brake = self.predict_vel(close_avg_y, far_avg_y)
            self.get_logger().info('calc_brake: "%s"' % calc_brake)

            p_vel = (1 - (self.vel / self.target_vel)) # increase proportionally as it approaches target
            if p_vel > 0:
                calc_throttle = self.max_throttle * p_vel
            
            elif p_vel <= 0: # if its over maximum, cut throttle
                calc_throttle = 0

            # calculate steering    
            # PID steering for close and far cone detection
            calc_steering = (self.steering_pc)*(close_avg_y)*abs(2*close_avg_y) \
                + (self.steering_ic)*(self.sum_close_avg_y + close_avg_y) \
                + (self.steering_dc)*(close_avg_y - self.prev_close_avg_y) \
                + (self.steering_pf)*far_avg_y*abs(2*far_avg_y) \
                + (self.steering_if)*(self.sum_far_avg_y + far_avg_y) \
                + (self.steering_df)*(far_avg_y - self.prev_far_avg_y) \
            # self.get_logger().info('steering: "%s"' % calc_steering)

            self.prev_close_avg_y = close_avg_y
            self.prev_far_avg_y = far_avg_y
            self.sum_close_avg_y += close_avg_y
            self.sum_far_avg_y += far_avg_y

            # define publishing data
            control_msg = ControlCommand()
            control_msg.throttle = float(calc_throttle)
            control_msg.steering = float(calc_steering)
            control_msg.brake =  0.0 #float(calc_brake)
            self.movement_publisher_.publish(control_msg)


        # self.get_logger().info('close cones: "%s"' % self.close_cones)
        # self.get_logger().info('avg: "%s"' % close_avg_y)
        # self.get_logger().info('vel: "%s"' % self.vel)

## main call
def main(args=None):
    rclpy.init(args=args)

    control = Controller()
    rclpy.spin(control)
    
    control.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
