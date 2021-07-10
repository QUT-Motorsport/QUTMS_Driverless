import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointCloud
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32

from .sub_module.simple_lidar import find_points

class LidarProcessing(Node):
    def __init__(self):
        super().__init__('lidar_processing')
        self.lidar_subscription = self.create_subscription(
            PointCloud2,
            '/fsds/lidar/Lidar2',
            self.lidar_callback,
            10)
        self.lidar_subscription  # prevent unused variable warning

        self.math_publisher = self.create_publisher(
            Float32, # will change if we return all cones back
            'math_output', 
            10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.avg = 0.0

        self.geo_subscription = self.create_subscription(
            PointCloud2,
            '/fsds/lidar/Lidar2',
            self.lidar_callback,
            10)
        self.lidar_subscription  # prevent unused variable warning



    def find_avg(self, cones):
        if len(cones) != 0:
            average_y = 0
            for cone in cones:
                average_y += cone[1]
            average_y = average_y / len(cones)

            return average_y
        
        else:
            return 0 


    def lidar_callback(self, pcl_msg):
        """ In here, we will call calculations to ideally get the 
        distance, angle, and reflectivity of the cones"""

        self.cones = find_points(pcl_msg) 
        # self.get_logger().info('close cones: "%s"' % self.cones)

        self.avg = self.find_avg(self.cones) 
        self.get_logger().info('avg: "%lf"' % self.avg)


    def timer_callback(self):

        # msg = Float32MultiArray()
        # msg.data = self.cones

        msg = Float32()
        msg.data = float(self.avg)

        self.math_publisher.publish(msg)

        # self.get_logger().info('found: "%s"' % self.avg)


""" 
        # example steering code right and left
        if self.left == 0:
            if self.i < 1:
                self.i += 0.1
            elif self.i >= 1:
                self.left = 1

        elif self.left == 1:
            if self.i > -1:
                self.i -= 0.1
            elif self.i <= -1:
                self.left = 0

        msg = Float32()
        msg.data = self.i
        self.math_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

 """

def main(args=None):
    rclpy.init(args=args)

    lidar_processing = LidarProcessing()
    rclpy.spin(lidar_processing)
    
    lidar_processing.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
