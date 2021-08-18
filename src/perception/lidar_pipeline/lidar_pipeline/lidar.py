# ROS2 libraries
import rclpy
from rclpy.node import Node
# ROS2 message libraries
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
# custom sim data message libraries
from qutms_msgs.msg import ConeScan, ConeData
# other python libraries
import numpy
import time

# import ROS function that has been ported to ROS2 by
# SebastianGrans https://github.com/SebastianGrans/ROS2-Point-Cloud-Demo
from .sub_module.read_pcl import read_points
# helper math function
from .sub_module.simple_lidar import find_cones
from .sub_module.ground_plane_estimation import init_points

class LidarDetection(Node):
    def __init__(self):
        super().__init__('lidar_pipe')
        ## creates subscriber to 'Lidar2' with type PointCloud2 that sends data to lidar_callback
        self.pcl_subscription_ = self.create_subscription(
            PointCloud2,
            '/fsds/lidar/Lidar1',
            self.pcl_callback,
            10)
        self.pcl_subscription_  # prevent unused variable warning
        self.cones = []

        ## creates publisher to 'control_command' with type ControlCommand
        self.scan_publisher_ = self.create_publisher(
            ConeScan,
            'lidar_output', 
            10)
        # creates timer for publishing commands
        self.timer_period = 0.001  # seconds
        self.timer = self.create_timer(self.timer_period, self.publisher)

        # cone point vars
        self.max_range_cutoff = 15 # m
        self.distance_cutoff = 0.1 # m


    ## callback for lidar data to be sent to. used to call funtion to find cone coords
    def pcl_callback(self, pcl_msg):
        """ In here, we will call calculations to ideally get the xyz location 
        and reflectivity of the cones"""
        
        # Convert the list of floats into a list of xyz coordinates
        pcl_array = numpy.array(list(read_points(pcl_msg)))

        # self.get_logger().info('cones: "%s"' % pcl_array)

        init_points(pcl_array)
        
        self.cones = find_cones(pcl_array, self.max_range_cutoff, self.distance_cutoff)

        time.sleep(15)


    def publisher(self):
        cone_scan = ConeScan()
        # head = Header()

        cone_data = []
        for i in range(len(self.cones)):
            cone = ConeData()
            cone.x = self.cones[i][0]
            cone.y = self.cones[i][1]
            cone.z = self.cones[i][2]
            cone.i = self.cones[i][3]
            cone_data.append(cone)
       
        # head.stamp = rospy.Time.now()
        # cone_scan.header = head

        cone_scan.data = cone_data

        self.scan_publisher_.publish(cone_scan)
        

## main call
def main(args=None):
    rclpy.init(args=args)

    node = LidarDetection()
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
