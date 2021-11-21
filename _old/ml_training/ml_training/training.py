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

# import helper point and cone processing module
from .sub_module.get_yaml import return_track


# define camera coords relative to car frame
CAM_LOCAL_POSE = [-0.5, 0, 1, 1] #m

TRACK_PATH = '/home/developer/driverless_ws/src/perception/ml_training/ml_training/track.txt'

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

        self.cones = return_track()

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
    # axis x,y,z angles = forward-backward, left-right, up-down
    # rotation is according along these axis
    def odom_callback(self, odom_msg):

        rot_x = odom_msg.pose.pose.orientation.x
        rot_y = odom_msg.pose.pose.orientation.y
        rot_z = odom_msg.pose.pose.orientation.z
        rot_w = odom_msg.pose.pose.orientation.w
        rot_quat = [rot_w, rot_x, rot_y, rot_z] #quaternion WRT map
        print(rot_quat)

        # z points up, x points forward
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        z = odom_msg.pose.pose.position.z
        car_coords = [x, y, z] #m WRT map
        print(car_coords)

        rot_mat = t3d.euler.quat2mat(rot_quat) #rad [x, y, z] WRT map

        car_mat = np.zeros((4,4))
        car_mat[0:3, 0:3] = rot_mat
        car_mat[3, 0:3] = car_coords
        car_mat[3, 3] = 1
        print("\ncar_mat: \n", car_mat)

        cam_coords = np.matmul(CAM_LOCAL_POSE, car_mat)

        cam_mat = np.zeros((4,4))
        cam_mat[0:3, 0:3] = rot_mat
        cam_mat[3, 0:4] = cam_coords
        cam_mat[3, 3] = 1
        print("\ncam_mat: \n", cam_mat)
        cam_mat = np.asmatrix(cam_mat)
        cam_mat_i = cam_mat.getI()

        print("\n")
        time.sleep(1)

        # for cone in self.cones:
        cone = self.cones[0]
        cone_on_cam = np.matmul(cone, cam_mat_i)
        cone_on_cam = cone_on_cam.tolist()
        x = cone_on_cam[0][0] / -cone_on_cam[0][2]
        y = cone_on_cam[0][1] / -cone_on_cam[0][2]


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
