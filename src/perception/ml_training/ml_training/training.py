# import ROS2 libraries
import rclpy
from rclpy.node import Node
# import ROS2 message libraries
from nav_msgs.msg import Odometry
from fs_msgs.msg import Track
# importcustom sim data message libraries
from qutms_msgs.msg import ConeScan, ConeData

# import other python libraries
import transforms3d as t3d
import numpy as np
import time


# define camera coords relative to car frame
CAM_LOCAL_X = -0.5 #m
CAM_LOCAL_Y = 0 #m
CAM_LOCAL_Z = 1 #m

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
        self.track_subscription_  # prevent unused variable warning
        self.got_track = False
        self.cones = list()

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
        # msg format:   Odometry/PoseWithCovariance/Pose/Point/
        #               - [x,y,z]
        #               Odometry/PoseWithCovariance/Pose/Quaternion/ 
        #               - [x,y,z,w]
        
        pose_x = odom_msg.pose.pose.position.x
        pose_y = odom_msg.pose.pose.position.x
        pose_z = odom_msg.pose.pose.position.x
        pose = [pose_x, pose_y, pose_z]
        print("\npose:", pose)

        rot_x = odom_msg.pose.pose.orientation.x
        rot_y = odom_msg.pose.pose.orientation.y
        rot_z = odom_msg.pose.pose.orientation.z
        rot_w = odom_msg.pose.pose.orientation.w
        rot = [rot_x, rot_y, rot_z, rot_w]
        print("\nrot:", rot)

        time.sleep(1)
    
    # callback function for track data
    def track_callback(self, track_msg):
        # msg format:   Track/Cone/Point/
        #               - [x,y,z]
        #               Track/Cone/
        #               - colour
        #                   uint8 BLUE=0
        #                   uint8 YELLOW=1
        #                   uint8 ORANGE_BIG=2
        #                   uint8 ORANGE_SMALL=3
        #                   uint8 UNKNOWN=4
        
        self.cones = list() # clear previous list
        if self.got_track == False:
            for i in range(len(track_msg)):
                curr_cone = track_msg[i] # define current cone in track list
                x = curr_cone.location.x
                y = curr_cone.location.y 
                z = curr_cone.location.z
                c = curr_cone.color

                self.cones.append([x,y,z,c])
            
            self.got_track = True


    ## publisher for processed cone data
    def publisher(self):
        None


## main call
def main(args=None):
    rclpy.init(args=args)

    node = LocationProcessing()
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
