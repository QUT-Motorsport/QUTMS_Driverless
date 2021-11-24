# Adapted in python from https://github.com/stereolabs/zed-ros2-examples/blob/master/tutorials/zed_depth_tutorial/src/zed_depth_sub_tutorial.cpp
# Author: Alastair Bradford

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

# ROS2 message libraries
from sensor_msgs.msg import Image

from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np

class ZED_Processing(Node):
    def __init__(self):
        super().__init__('depth_processing')
        # create a quality of service profile compatible with zed node
        qos_profile = QoSProfile(10)
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST

        # creates subscriber to 'right_image' with type Image that sends data to right_callback
        self.r_cam_subscription_ = self.create_subscription(
            Image,
            'right_image',
            self.right_callback,
            10)
        self.r_cam_subscription_  # prevent unused variable warning

        # creates subscriber to 'left_image' with type Image that sends data to left_callback
        self.l_cam_subscription_ = self.create_subscription(
            Image,
            'left_image',
            self.left_callback,
            10)
        self.l_cam_subscription_  # prevent unused variable warning


        self.br = CvBridge()

    def right_callback(self, r_cam_msg):
        current_frame =  self.br.imgmsg_to_cv2(r_cam_msg)
        cv2.imshow("right camera", current_frame) # image with bounding boxes
        cv2.waitKey(1)

    def left_callback(self, l_cam_msg):
        current_frame =  self.br.imgmsg_to_cv2(l_cam_msg)
        cv2.imshow("left camera", current_frame) # image with bounding boxes
        cv2.waitKey(1)


## main call
def main(args=None):
    rclpy.init(args=args)

    node = ZED_Processing()
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
