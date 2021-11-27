# import ROS2 libraries
import rclpy
from rclpy.node import Node
# import ROS2 message libraries
from geometry_msgs.msg import TwistStamped
# import custom sim data message libraries
from fs_msgs.msg import ControlCommand
from qutms_msgs.msg import ConeScan, ConeData

# import helper drive processing module
from .motion_proc import * 

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')
        ## creates subscriber to 'lidar_processed' with type ConeScan that sends data to lidar_callback
        self.lidar_subscription_ = self.create_subscription(
            ConeScan,
            'lidar_processed',
            self.lidar_callback,
            10)
        self.lidar_subscription_ 

        ## creates subscriber to 'cam_processed' with type ConeScan that sends data to cam_callback
        self.cam_subscription_ = self.create_subscription(
            ConeScan,
            'cam_processed',
            self.cam_callback,
            10)
        self.cam_subscription_ 

        ## creates subscriber to 'gss' with type TwistStamped that sends data to geo_callback
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
        self.timer = self.create_timer(self.timer_period, self.publisher)

        ## initial class variables
        self.time = 0
        self.close_cones = list()
        self.far_cones = list()
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

        # PID coeffs
        self.steering_pc = 0.057
        self.steering_ic = 0
        self.steering_dc = 0.85
        self.steering_pf = 0.009
        self.steering_if = 0
        self.steering_df = 0.45

        # accel + vel targets
        self.vel_p = 0.7
        self.max_throttle = 0.2 # m/s^2
        self.max_vel = 1.4 # m/s
        self.target_vel = self.max_vel # initially target is max
        self.min_vel = self.max_vel / 2
        self.brake_p = 0.1


    ## callback for lidar processed data to be sent to
    def lidar_callback(self, cone_scan):
        """ In here, we will call calculations to ideally get the xyz location 
        and reflectivity of the cones"""
        self.close_cones = list() # init empty lists
        self.far_cones = list()

        # retrieve message data
        cones = cone_scan.data
        # calculate distance between lidar and cone
        for j in range(0, len(cones)):
            cone = list()
            cone.append(cones[j].x)
            cone.append(cones[j].y)
            cone.append(cones[j].z)
            cone.append(cones[j].i)

            # check which bin cone falls in
            # further cones will have less impact on immediate decisions
            if distance(0, 0, cone[0], cone[1]) < self.close_range_cutoff:
                self.close_cones.append(cone)
            if distance(0, 0, cone[0], cone[1]) < self.far_range_cutoff:
                self.far_cones.append(cone)


    ## callback for camera processed data to be sent to
    def cam_callback(self, cone_scan):
        """ In here, we will call calculations to ideally get the xyz location 
        and reflectivity of the cones"""
        self.close_cones = list() # init empty lists
        self.far_cones = list()

        # retrieve message data
        cones = cone_scan.data
        # calculate distance between lidar and cone
        for j in range(0, len(cones)):
            cone = list()
            cone.append(cones[j].x)
            cone.append(-(cones[j].y))
            cone.append(cones[j].z)
            cone.append(cones[j].c)

            # check which bin cone falls in
            # further cones will have less impact on immediate decisions
            if distance(0, 0, cone[0], cone[1]) < self.close_range_cutoff:
                self.close_cones.append(cone)
            if distance(0, 0, cone[0], cone[1]) < self.far_range_cutoff:
                self.far_cones.append(cone)


    ## callback for geometry data to be sent to. used to find car's velocity
    def geo_callback(self, geo_msg):
        vel_x = geo_msg.twist.linear.x
        vel_y = geo_msg.twist.linear.y

        self.vel = math.sqrt(vel_x*vel_x + vel_y*vel_y)


    ## callback for publishing FSDS command messages at specific times
    def publisher(self):
        self.time += self.timer_period # increase time taken

        # initialise variables
        calc_throttle = 0.0 
        calc_brake = 0.0
        calc_steering = 0.0

        # call cone detection
        # frame of reference is flipped along FOV (for some reason)
        close_avg_y = find_avg_y(self.close_cones) 
        far_avg_y = find_avg_y(self.far_cones)

        # wait for initial publishing delay
        if (self.time >= 3): 
            # calculate throttle + brake
            [calc_brake, self.target_vel] = predict_vel(close_avg_y, far_avg_y, \
                self.brake_p, self.min_vel, self.max_vel, self.vel_p)

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
            
            self.prev_close_avg_y = close_avg_y
            self.prev_far_avg_y = far_avg_y
            self.sum_close_avg_y += close_avg_y
            self.sum_far_avg_y += far_avg_y

            # define publishing data
            control_msg = ControlCommand()
            control_msg.throttle = float(calc_throttle)
            control_msg.steering = float(calc_steering)
            control_msg.brake =  float(calc_brake)
            self.movement_publisher_.publish(control_msg)


## main call
def main(args=None):
    rclpy.init(args=args)

    node = SimpleController()
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
