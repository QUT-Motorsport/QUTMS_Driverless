# import ROS2 libraries
import rclpy
from rclpy.node import Node

# import ROS2 message libraries
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
# import custom sim data message libraries
from fs_msgs.msg import Track
from qutms_msgs.msg import ConeScan, ConeData

# import other python libraries
import transforms3d as t3d
import numpy as np
import time
import math


# define camera coords relative to car frame
CAM_LOCAL_POSE = [-0.5, 0, 1] #m
CAM_LOCAL_ROT = [0, 0, math.pi] #rads

class LocationProcessing(Node):
    def __init__(self):
        super().__init__('location_processing')

        ## creates subscriber to 'odom' topic with type Odometry that sends data to odom_callback
        self.odom_subscription_ = self.create_subscription(
            Odometry,
            '/fsds/testing_only/odom',
            self.odom_callback,
            10)
        self.odom_subscription_  # prevent unused variable warning

        ## creates subscriber to 'track' topic with type Track that sends data to track_callback
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
    # msg format:   Odometry/PoseWithCovariance/Pose/Point/
    #               - [x,y,z]
    #               Odometry/PoseWithCovariance/Pose/Quaternion/ 
    #               - [x,y,z,w]

    # axis x,y,z angles = up-down, left-right, forward-backward
    # rotation is according along these axis
    def odom_callback(self, odom_msg):
        
        pose_x = odom_msg.pose.pose.position.x
        pose_y = odom_msg.pose.pose.position.y
        pose_z = odom_msg.pose.pose.position.z
        car_pose = [pose_x, pose_y, pose_z] #m WRT map
        # print("\ncar_pose: ", car_pose)

        rot_x = odom_msg.pose.pose.orientation.x
        rot_y = odom_msg.pose.pose.orientation.y
        rot_z = odom_msg.pose.pose.orientation.z
        rot_w = odom_msg.pose.pose.orientation.w
        car_rot = [rot_x, rot_y, rot_z, rot_w] #quaternion WRT map

        angles = t3d.euler.quat2euler(car_rot) #rad [x, y, z] WRT map
        # print("\ncar_rot: ", angles)

        # time.sleep(1)


    # callback function for track data
    # msg format:   Track/Cone/Point/
    #               - [x,y,z]
    #               Track/Cone/
    #               - colour
    #                   uint8 BLUE=0
    #                   uint8 YELLOW=1
    #                   uint8 ORANGE_BIG=2
    #                   uint8 ORANGE_SMALL=3
    #                   uint8 UNKNOWN=4
    def track_callback(self, track_msg):
        print("here")
        # if self.got_track == False:
        # cones_data = track_msg.track
        # for i in range(len(cones_data)):
        #     curr_cone = track_msg[i] # define current cone in track list
        #     print(curr_cone)
        #     x = curr_cone.location.x
        #     y = curr_cone.location.y 
        #     z = curr_cone.location.z
        #     c = curr_cone.color

        #     self.cones.append([x,y,z,c])
        
        # self.got_track = True


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
