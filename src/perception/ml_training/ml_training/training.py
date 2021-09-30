# import ROS2 libraries
import rclpy
from rclpy.node import Node
# import ROS2 message libraries
from nav_msgs.msg import Odometry
from fs_msgs.msg import Track
# importcustom sim data message libraries
from qutms_msgs.msg import ConeScan, ConeData

# import other python libraries
import numpy as np
import time


class LocationProcessing(Node):
    def __init__(self):
        super().__init__('locationz_processing')

        ## creates subscriber to 'odom' with type Image that sends data to odom_callback
        self.odom_subscription_ = self.create_subscription(
            Odometry,
            '/fsds/testing_only/odom',
            self.odom_callback,
            10)
        self.odom_subscription_  # prevent unused variable warning

        ## creates subscriber to 'odom' with type Image that sends data to odom_callback
        self.track_subscription_ = self.create_subscription(
            Track,
            '/fsds/testing_only/track',
            self.track_callback,
            10)
        self.odom_subscription_  # prevent unused variable warning

        ## creates publisher to 'control_command' with type ControlCommand
        # self.scan_publisher_ = self.create_publisher(
        #     ConeScan,
        #     'cam_processed', 
        #     10)
        # # creates timer for publishing commands
        # self.timer_period = 0.001  # seconds
        # self.timer = self.create_timer(self.timer_period, self.publisher)


    # callback function for odom processing
    def odom_callback(self, odom_msg):
        None

    
    # callback function for track data
    def track_callback(self, track_msg):
        None
        


    ## publisher for processed cone data
    def publisher(self):
        cone_scan = ConeScan()
        # head = Header()

        cone_data = list()
        for i in range(len(self.cone_coords)):
            cone = ConeData()
            cone.x = self.cone_coords[i][0]
            cone.y = self.cone_coords[i][1]
            cone.z = self.cone_coords[i][2]
            cone.c = self.cone_coords[i][3]
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

    node = CamProcessing()
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
