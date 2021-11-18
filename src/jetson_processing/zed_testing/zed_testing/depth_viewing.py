# Adapted in python from https://github.com/stereolabs/zed-ros2-examples/blob/master/tutorials/zed_depth_tutorial/src/zed_depth_sub_tutorial.cpp
# Author: Alastair Bradford

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

# ROS2 message libraries
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage

import cv2 # OpenCV library
import numpy as np

from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

class Depth_Processing(Node):
    def __init__(self):
        super().__init__('depth_processing')
        # create a quality of service profile compatible with zed node
        qos_profile = QoSProfile(depth=1)
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST

        # creates subscriber to 'depth' with type Image that sends data to depth_callback
        self.disp_subscription_ = self.create_subscription(
            DisparityImage,
            '/zed2i/zed_node/disparity/disparity_image',
            self.disp_callback,
            qos_profile)
        self.disp_subscription_  # prevent unused variable warning

        # creates subscriber to 'depth' with type Image that sends data to depth_callback
        self.depth_subscription_ = self.create_subscription(
            Image,
            '/zed2i/zed_node/depth/depth_registered',
            self.depth_callback,
            qos_profile)
        self.depth_subscription_  # prevent unused variable warning

        self.br = CvBridge()

    def disp_callback(self, disp_msg):
        print("received")
        disp_img = disp_msg.image
        current_frame =  self.br.imgmsg_to_cv2(disp_img)
        cv2.imshow("depth camera", current_frame)
        cv2.waitKey(1)


## main call
def main(args=None):
    rclpy.init(args=args)

    node = Depth_Processing()
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
