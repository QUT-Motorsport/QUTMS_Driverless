# import ROS2 libraries
import rclpy
from rclpy.node import Node
# import ROS2 message libraries
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
# import custom sim data message libraries
from qutms_msgs.msg import ConeScan, ConeData
# other python libraries
import numpy
import time

# import ROS function that has been ported to ROS2 by
# SebastianGrans https://github.com/SebastianGrans/ROS2-Point-Cloud-Demo
from .sub_module.read_pcl import *
# helper math function
from .sub_module.ground_plane_estimation import lidar_main

LIDAR_NODE = '/fsds/lidar/Lidar1'
# LIDAR_NODE = '/velodyne_points'

class LidarProcessing(Node):
    def __init__(self):
        super().__init__('lidar_processing')
        ## creates subscriber to 'Lidar2' with type PointCloud2 that sends data to lidar_callback
        self.pcl_subscription_ = self.create_subscription(
            PointCloud2,
            LIDAR_NODE,
            self.pcl_callback,
            10)
        self.pcl_subscription_  # prevent unused variable warning
        self.cones = list()

        if LIDAR_NODE == '/fsds/lidar/Lidar1':
            print("LiDAR Processing for FSDS")
        elif LIDAR_NODE == '/velodyne_points':
            print("LiDAR Processing for Velodyne")

        ## creates publisher to 'lidar_output' with type ConeScan
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
        
        start = time.time()
        # Convert the list of floats into a list of xyz coordinates

        pcl_array = read_points_list(pcl_msg)
        pcl_list = list()
        for point in pcl_array:
            if LIDAR_NODE == '/fsds/lidar/Lidar1':
                pcl_list.append([point[0], point[1], point[2]])
            elif LIDAR_NODE == '/velodyne_points':
                pcl_list.append([point[0], point[1], point[2], point[3]])

        #print("convert time: ", time.time()-start)

        pcl_list = numpy.array(list(read_points(pcl_msg)))

        # pcl_list = pcl_array.tolist()
        # textfile = open("/home/developer/datasets/16k_points.txt", "w")
        # textfile.write(str(pcl_list))
        # textfile.close()

        # calls first module from ground estimation algorithm
        self.cones = lidar_main(pcl_list.tolist(), False, False, "/home/developer/datasets/figures") 
        #print('cones:', self.cones)

        print("Total Time:", time.time()-start)


    def publisher(self):
        cone_scan = ConeScan() # define msg class
        # head = Header()

        cone_data = []
        for i in range(len(self.cones)): # define msg component elements 
            cone = ConeData() 
            cone.x = self.cones[i][0] # x, y, z coords + intensity
            cone.y = self.cones[i][1]
            cone.z = 0.2 #self.cones[i][2]
            cone.i = 3.0 #self.cones[i][3]
            cone_data.append(cone) # add cone to msg list
       
        # head.stamp = rospy.Time.now()
        # cone_scan.header = head

        cone_scan.data = cone_data # add list of cones to msg data element

        self.scan_publisher_.publish(cone_scan) # publish cone data
        

## main call
def main(args=None):
    print("| STARTING | lidar.py: main()")
    rclpy.init(args=args)

    node = LidarProcessing()
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()