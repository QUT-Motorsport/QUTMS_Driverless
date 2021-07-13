import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TwistStamped

from std_msgs.msg import Float32MultiArray
from fs_msgs.msg import ControlCommand
from qutms_msgs.msg import ConeData, Location

from .sub_module.simple_lidar import find_points
import math

class Controller(Node):
    def __init__(self):
        super().__init__('control')
        self.lidar_subscription_ = self.create_subscription(
            PointCloud2,
            '/fsds/lidar/Lidar2',
            self.lidar_callback,
            10)
        self.lidar_subscription_  # prevent unused variable warning
        self.cone_points = []

        self.geo_subscription_ = self.create_subscription(
            TwistStamped,
            '/fsds/gss',
            self.geo_callback,
            10)
        self.geo_subscription_ 
        self.vel = 0.0

        self.movement_publisher_ = self.create_publisher(
            ControlCommand, # will change if we return all cones back
            '/fsds/control_command', 
            10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.pub_callback)
        self.sum_avg_x = 0
        self.sum_avg_y = 0
        self.prev_avg_x = 0
        self.prev_avg_y = 0


    def find_avg(self):
        length = len(self.cone_points)
        if length != 0:
            average_x = 0
            average_y = 0
            for cone in self.cone_points:
                average_x += cone[0]
                average_y += cone[1]
            average_x = average_x / length
            average_y = average_y / length

            return [average_x, average_y]
        
        else:
            return [0, 0] 


    def lidar_callback(self, pcl_msg):
        """ In here, we will call calculations to ideally get the 
        distance, angle, and reflectivity of the cones"""
        cones_range_cutoff = 12 # m      can tweak
        distance_cutoff = 0.1 # m       can tweak

        self.cone_points = find_points(pcl_msg, cones_range_cutoff, distance_cutoff) 
        

    def geo_callback(self, geo_msg):
        vel_x = geo_msg.twist.linear.x
        vel_y = geo_msg.twist.linear.y

        self.vel = math.sqrt(vel_x*vel_x + vel_y*vel_y)


    def pub_callback(self):
        avg = self.find_avg()
        avg_x = avg[0]
        avg_y = avg[1]

        # Calculate throttle
        max_throttle = 0.2
        target_vel = 4
        calc_throttle = 0.0

        p_vel = (1 - (self.vel / target_vel))
        if p_vel > 0:
            calc_throttle = max_throttle * p_vel
        
        elif p_vel <= 0:
            calc_throttle = 0
        
        # Calculate steering
        steering_p = 2
        steering_i = 0.01
        steering_d = 1.5
      
        calc_steering = (-steering_p)*(avg_y) + (-steering_i)*(self.sum_avg_y + avg_y) + (-steering_d)*(avg_y - self.prev_avg_y)
        if calc_steering > 1:
            calc_steering = 1
        
        elif calc_steering < -1:
            calc_steering = -1

        self.prev_avg_y = avg_y
        self.sum_avg_y += avg_y

        control_msg = ControlCommand()
        control_msg.throttle = float(calc_throttle)
        control_msg.steering = float(calc_steering)
        control_msg.brake = 0.0
        self.movement_publisher_.publish(control_msg)


        # self.get_logger().info('close cones: "%s"' % self.cone_points)
        self.get_logger().info('avg: "%s"' % avg_y)


def main(args=None):
    rclpy.init(args=args)

    control = Controller()
    rclpy.spin(control)
    
    control.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
