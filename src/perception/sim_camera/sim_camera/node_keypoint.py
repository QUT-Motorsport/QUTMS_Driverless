# import ROS2 libraries
import os
import time

import cv2
from cv_bridge import CvBridge  # package to convert between ROS and OpenCV Images

# import other python libraries
import numpy as np

# importcustom sim data message libraries
from qutms_msgs.msg import ConeData, ConeScan
import rclpy
from rclpy.node import Node

# import ROS2 message libraries
from sensor_msgs.msg import Image

# import helper cone location processing module
from .depth_proc import *

# import helper image processing module
from .img_processing_KP import *


class CamProcessing(Node):
    def __init__(self):
        super().__init__("camera_processing")

        ## creates subscriber to 'cam1' with type Image that sends data to cam_callback
        self.cam_subscription_ = self.create_subscription(
            Image,
            # '/fsds/camera/cam1', # 785x785 (square defaults)
            "/fsds/camera/cam2",  # 1080p (like a usual camera)
            self.cam_callback,
            10,
        )
        self.cam_subscription_  # prevent unused variable warning
        self.br = CvBridge()
        self.cone_coords = list()

        ## creates publisher to 'control_command' with type ControlCommand
        self.scan_publisher_ = self.create_publisher(ConeScan, "cam_processed", 10)
        # creates timer for publishing commands
        self.timer_period = 0.001  # seconds
        self.timer = self.create_timer(self.timer_period, self.publisher)

    # callback function for camera image processing
    def cam_callback(self, cam_msg):
        # get image
        raw_frame = self.br.imgmsg_to_cv2(cam_msg)
        h, w, _ = raw_frame.shape

        cones = cam_main(raw_frame, False)

        # call return_locations to return xyzc for every cone
        # self.cone_coords = return_locations(w/2, cones) # send cones for distance calculation

        # print("*"*20)
        # print(cones)
        # cv2.imshow("1", raw_frame)
        # cv2.waitKey(1)

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


if __name__ == "__main__":
    main()
