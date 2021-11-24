import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TwistStamped

from std_msgs.msg import Float32MultiArray

from qutms_msgs.msg import ConeData, Location

from .sub_module.simple_lidar import find_points
import math

class LidarProcessing(Node):
    def __init__(self):
        super().__init__('lidar_processing')
        self.lidar_subscription_ = self.create_subscription(
            PointCloud2,
            '/fsds/lidar/Lidar2',
            self.lidar_callback,
            10)
        self.lidar_subscription_  # prevent unused variable warning
        self.cone_points = []
        self.length = 0.0

        self.math_publisher_ = self.create_publisher(
            ConeData, # will change if we return all cones back
            'math_output', 
            10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.pub_callback)

        self.geo_subscription_ = self.create_subscription(
            TwistStamped,
            '/fsds/gss',
            self.geo_callback,
            10)
        self.geo_subscription_ 
        self.vel = 0.0


    def find_avg(self, cone_points):
        if len(cone_points) != 0:
            average_y = 0
            for cone in cone_points:
                average_y += cone[1]
            average_y = average_y / len(cone_points)

            return average_y
        
        else:
            return 0 


    def lidar_callback(self, pcl_msg):
        """ In here, we will call calculations to ideally get the 
        distance, angle, and reflectivity of the cones"""
        cones_range_cutoff = 7 # m      can tweak
        distance_cutoff = 0.1 # m       can tweak

        self.cone_points = find_points(pcl_msg, cones_range_cutoff, distance_cutoff) 
        # self.get_logger().info('close cones: "%s"' % self.cone_points)

        self.length = len(self.cone_points)


    def geo_callback(self, geo_msg):

        # max_throttle = 0.2
        # target_vel = 4

        vel_x = geo_msg.twist.linear.x
        vel_y = geo_msg.twist.linear.y

        self.vel = math.sqrt(vel_x*vel_x + vel_y*vel_y)

    def pub_callback(self):

        cones = ConeData()
        cones.array_len = float(self.length)
        cones.car_vel = float(self.vel)

        for i in range(len(self.cone_points)):
            cone = Location()
            cone.cone_point = self.cone_points[i]
            cones.cone_array[i] = cone

        self.get_logger().info('velocity: "%s"' % self.vel)

        self.math_publisher_.publish(cones)



def main(args=None):
    rclpy.init(args=args)

    lidar_processing = LidarProcessing()
    rclpy.spin(lidar_processing)
    
    lidar_processing.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
