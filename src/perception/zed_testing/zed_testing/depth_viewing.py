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
import time
import numpy as np

from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

from .sub_module.img_processing import * # import image processing func

class Depth_Processing(Node):
    def __init__(self):
        super().__init__('depth_processing')
        # create a quality of service profile compatible with zed node
        qos_profile = QoSProfile(depth=1)
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST

        # creates subscriber to 'image_rect_color' with type Image that sends data to stereo_callback
        self.stereo_subscription_ = self.create_subscription(
            Image,
            '/zed2i/zed_node/stereo/image_rect_color',
            self.stereo_callback,
            qos_profile)
        self.stereo_subscription_  # prevent unused variable warning

        # creates subscriber to 'disparity_image' with type DisparityImage that sends data to disp_callback
        self.disp_subscription_ = self.create_subscription(
            DisparityImage,
            '/zed2i/zed_node/disparity/disparity_image',
            self.disp_callback,
            qos_profile)
        self.disp_subscription_  # prevent unused variable warning

        self.br = CvBridge()
        self.found_cones = list()
        self.display = True
        self.bounding_box_frame = list()

        self.count = 0


    # callback function for camera image processing
    def stereo_callback(self, cam_msg):
        # get image
        raw_frame = self.br.imgmsg_to_cv2(cam_msg)
        self.found_cones, self.bounding_box_frame, left_frame = cam_main(raw_frame)

        h, w, _ = left_frame.shape

        if self.count % 15 == 0 and self.count > 300:
            file_name = "/home/developer/datasets/annotation/annotate_" + str(self.count) + ".txt"
            img_name = "/home/developer/datasets/annotation/annotate_" + str(self.count) + ".png"
            textfile = open(file_name, "w")

            for cone in self.found_cones:
                x_norm = cone[0] / w
                y_norm = cone[1] / h
                w_norm = cone[4] / w
                h_norm = cone[5] / h
                coords = "0 " + str(x_norm) + " " + str(y_norm) + " " + str(w_norm) + " " + str(h_norm) + "\n"
                textfile.write(coords)

            textfile.close()

            cv2.imwrite(img_name, left_frame)
            cv2.imshow('left', left_frame)
            cv2.waitKey(1)

        self.count += 1
            

    def disp_callback(self, disp_msg):
        disp_img = disp_msg.image # disparity image of floats
        focal = disp_msg.f # focal len in pixels
        base = disp_msg.t # baseline

        current_frame = self.br.imgmsg_to_cv2(disp_img, desired_encoding='32FC1')

        for cone in self.found_cones:
            disparity = current_frame[cone[1], cone[0]]
            depth = -focal * base / disparity

            string = str(round(depth, 3)) + "m"
            cv2.putText(
                self.bounding_box_frame, string, # frame to draw on, label to draw
                (cone[2], cone[3]-3), # position
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, # font, font size
                (0, 255, 0), 1) # red font colour, thickness 1

        if self.display == True and self.bounding_box_frame != []:
            # show bounding boxes
            cv2.imshow("depth", self.bounding_box_frame) # image with bounding boxes
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
