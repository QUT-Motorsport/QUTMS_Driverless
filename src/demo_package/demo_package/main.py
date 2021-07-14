# ROS2 libraries
import rclpy
from rclpy.node import Node
# ROS2 message libraries
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32MultiArray
# custom sim data message libraries
from fs_msgs.msg import ControlCommand
from qutms_msgs.msg import ConeData, Location
# helper math function
from .sub_module.simple_lidar import find_points
import math

class Controller(Node):
    def __init__(self):
        super().__init__('control')
        ## creates subscriber to 'Lidar2' with type PointCloud2 that sends data to lidar_callback
        self.lidar_subscription_ = self.create_subscription(
            PointCloud2,
            '/fsds/lidar/Lidar2',
            self.lidar_callback,
            10)
        self.lidar_subscription_  # prevent unused variable warning

        ## creates subscriber to 'gss' with type TwistStamped that sends data to lidar_callback
        self.geo_subscription_ = self.create_subscription(
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
        self.cone_points = []
        self.sum_avg_x = 0 # integral components
        self.sum_avg_y = 0 
        self.prev_avg_x = 0 # derivative components
        self.prev_avg_y = 0
        self.vel = 0.0

        ## test demo variables to change

        ## MODIFY THESE AND MAKE COPIES OF WORKING CONFIGURATIONS
        # cone point vars
        self.cones_range_cutoff = 7 # m
        self.distance_cutoff = 0.1 # m

        # PID coeffs
        self.steering_p = 0.5
        self.steering_i = 0
        self.steering_d = 1.1

        # accel + vel targets
        self.max_throttle = 0.2 # m/s^2
        self.max_vel = 7 # m/s
        self.target_vel = self.max_vel  # initially target is max

        ## DEMO 2.2 CONSTANTS
        # # cone point vars
        # self.cones_range_cutoff = 7 # m
        # self.distance_cutoff = 0.1 # m

        # # PID coeffs
        # self.steering_p = 0.5
        # self.steering_i = 0
        # self.steering_d = 1.1

        # # accel + vel targets
        # self.max_throttle = 0.2 # m/s^2
        # self.max_vel = 5 # m/s
        # self.target_vel = self.max_vel  # initially target is max
        
        ## DEMO 2.1 CONSTANTS - FROM VIDEO
        # # cone point vars
        # self.cones_range_cutoff = 6 # m
        # self.distance_cutoff = 0.1 # m

        # # PID coeffs
        # self.steering_p = 0.8 
        # self.steering_i = 0
        # self.steering_d = 1.7

        # # accel + vel targets
        # self.max_throttle = 0.2 # m/s^2
        # self.max_vel = 4 # m/s
        # self.target_vel = self.max_vel # initially target is max

    # callback for lidar data to be sent to. used to call funtion to find cone coords
    def lidar_callback(self, pcl_msg):
        """ In here, we will call calculations to ideally get the 
        distance, angle, and reflectivity of the cones"""
        # 
        self.cone_points = find_points(pcl_msg, self.cones_range_cutoff, self.distance_cutoff) 
        
    # callback for geometry data to be sent to. used to find var's current velocity
    def geo_callback(self, geo_msg):
        vel_x = geo_msg.twist.linear.x
        vel_y = geo_msg.twist.linear.y

        self.vel = math.sqrt(vel_x*vel_x + vel_y*vel_y)

    # helper function to find the average y "centre" of the cones. this is calculated wrt the FOV centre
    def find_avg_y(self):
        length = len(self.cone_points)
        if length != 0:
            average_y = 0
            for cone in self.cone_points:
                average_y += cone[1]
            average_y = average_y / length

            return average_y
        
        else:
            return 0 

    # helper function to adjust velocity based on how tight the turn is ahead
    def predict_vel(self, avg_y):
        length = len(self.cone_points)
        if length != 0:
            current_max_y = 0
            element = 0
            max_element = 0
            for cone in self.cone_points:
                cone_y = abs(cone[1])
                if cone_y > current_max_y: # if this coord is greater than the current max
                    current_max_y = cone_y # replace current maxs
                    max_element = element # record array index
                element += 1

            targ_cone = self.cone_points[max_element]
            cone_x = targ_cone[0]
            cone_y = targ_cone[1]

            if ( cone_y > 1.5*avg_y ) and ( abs(avg_y) > 0.5 ) and (cone_x > 1):
                # self.get_logger().info('cone_y: "%s"' % cone_y)
                # self.get_logger().info('cone_x: "%s"' % cone_x)

                self.target_vel = (1 / cone_x) * (self.cones_range_cutoff / 2) * 2
                self.get_logger().info('max_vel: "%s"' % self.target_vel)

                if self.target_vel < self.max_vel / 2:
                    self.target_vel = self.max_vel / 2 # stop it from slowing too much
                
                # self.get_logger().info('max_vel: "%s"' % self.target_vel)

            else:
                self.target_vel = self.max_vel


    # callback for publishing FSDS command messages at specific times
    def pub_callback(self):
        self.time += self.timer_period # increase time taken

        avg_y = -(self.find_avg_y()) # frame of reference is flipped along FOV (for some reason)
        # self.get_logger().info('avg_y: "%s"' % avg_y)

        # Calculate throttle
        calc_throttle = 0.0 # initialise throttle

        self.predict_vel(avg_y)
        
        p_vel = (1 - (self.vel / self.target_vel)) # increase proportionally as it approaches target
        if p_vel > 0:
            calc_throttle = self.max_throttle * p_vel
        
        elif p_vel <= 0: # if its over maximum, cut throttle
            calc_throttle = 0
        
        # Calculate steering    
        if (self.time >= 3): # wait for initial publishing delay
            
            # PID steering 
            calc_steering = (self.steering_p)*(avg_y) + (self.steering_i)*(self.sum_avg_y + avg_y) + (self.steering_d)*(avg_y - self.prev_avg_y)
            # self.get_logger().info('steering: "%s"' % calc_steering)

            # ensure limit isnt reached
            if calc_steering > 1:
                calc_steering = 1
            
            elif calc_steering < -1:
                calc_steering = -1

            self.prev_avg_y = avg_y
            self.sum_avg_y += avg_y

            # define publishing data
            control_msg = ControlCommand()
            control_msg.throttle = float(calc_throttle)
            control_msg.steering = float(calc_steering)
            control_msg.brake = 0.0
            self.movement_publisher_.publish(control_msg)


        # self.get_logger().info('close cones: "%s"' % self.cone_points)
        # self.get_logger().info('avg: "%s"' % avg_y)
        # self.get_logger().info('vel: "%s"' % self.vel)


def main(args=None):
    rclpy.init(args=args)

    control = Controller()
    rclpy.spin(control)
    
    control.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
