# import ROS2 libraries
import rclpy
from rclpy.node import Node
# import ROS2 message libraries
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
# import custom sim data message libraries
from qutms_msgs.msg import ConeScan, ConeData

# import other python libraries
import numpy as np

# import helper point and cone processing module
from .sub_module.point_processing import find_cones
# import ROS function that has been ported to ROS2 by SebastianGrans
# https://github.com/SebastianGrans/ROS2-Point-Cloud-Demo
from .sub_module.read_pcl import read_points


class LidarProcessing(Node):
    def __init__(self):
        super().__init__('lidar_processing')
        ## creates subscriber to 'Lidar2' with type PointCloud2 that sends data to lidar_callback
        self.pcl_subscription_ = self.create_subscription(
            PointCloud2,
            '/fsds/lidar/Lidar2',
            self.pcl_callback,
            10)
        self.pcl_subscription_  # prevent unused variable warning
        self.cones = list()

        ## creates publisher to 'control_command' with type ControlCommand
        self.scan_publisher_ = self.create_publisher(
            ConeScan,
            'lidar_processed', 
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
        pcl_array = np.array(list(read_points(pcl_msg)))

        # call find_cones to retrieve cone x,y,z + intensity
        self.cones = find_cones(pcl_array, self.max_range_cutoff, self.distance_cutoff)
        
        
    ## publisher for processed cone data
    def publisher(self):
        cone_scan = ConeScan()
        # head = Header()

        cone_data = list()
        for i in range(len(self.cones)):
            cone = ConeData()
            cone.x = self.cones[i][0]
            cone.y = self.cones[i][1]
            cone.z = self.cones[i][2]
            cone.c = self.cones[i][3]
            cone_data.append(cone)
              
        # cant work out how headers work lmao, this breaks stuff
        # head.stamp = rospy.Time.now()
        # head.frame_id = "lidar2"
        # cone_scan.header = head

        cone_scan.data = cone_data

        self.scan_publisher_.publish(cone_scan)
        

## main call
def main(args=None):
    rclpy.init(args=args)

    node = LidarProcessing()
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
