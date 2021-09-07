# ROS2 libraries
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
# ROS2 message libraries
from sensor_msgs.msg import Image
# custom sim data message libraries
from qutms_msgs.msg import ConeScan
# other python libraries
import numpy as np
import cv2 # OpenCV library
# helper processing module
from .sub_module.processing import *
import sys
from pprint import pprint

#############################
##### utility functions #####
#############################

class Cam_Pipe(Node):
    def __init__(self):
        super().__init__('control')

         # creates subscriber to 'cam1' with type Image that sends data to cam_callback
        self.cam_subscription_ = self.create_subscription(
            Image,
            '/fsds/camera/cam2',
            self.cam_callback,
            10)
        self.cam_subscription_  # prevent unused variable warning

        self.br = CvBridge()

    # callback function for camera image processing
    def cam_callback(self, cam_msg):
        # get image
        current_frame = self.br.imgmsg_to_cv2(cam_msg)


        # mask off sky and car
        roi_mask = np.ones(current_frame.shape[:2], dtype="uint8")
        cv2.rectangle(roi_mask, (0, 0), (current_frame.shape[1], 400), 0, -1) ,[current_frame.shape[2]-60,500] # mask off sky
        a = 30 # variables for the position of car mask
        b = 100
        c = 235
        h1 = 80
        h2 = 155
        cv2.fillPoly(roi_mask, [np.array([ # mask off car
            [a,current_frame.shape[1]],
            [b, current_frame.shape[1]-h1],
            [c, current_frame.shape[1]-h2],
            [current_frame.shape[0]/2,585],
            [current_frame.shape[0]-c,current_frame.shape[1]-h2],
            [current_frame.shape[0]-b,current_frame.shape[1]-h1],
            [current_frame.shape[0]-a,current_frame.shape[1]]
            ], np.int32)], 0)
        current_frame = cv2.bitwise_and(current_frame, current_frame, mask=roi_mask)

        # used to crop frame to only required area
        h, w, _ = current_frame.shape
        current_frame =  current_frame[int(h/2):int(h*9/10), 0:w]# made variable to cam resolution
        # vertical res: halfway down to 9/10ths down
        # horizontal res: full

        bounding_box_frame = np.copy(current_frame)

        # convert to hsv
        current_hsv = cv2.cvtColor(bounding_box_frame, cv2.COLOR_BGR2HSV)

        # passable variables for colours
        orange = 0
        yellow = 1
        blue = 2

        # retrieves the mask and edges for each colour
        # from sub module function
        [orange_mask, orange_edges] = feature_extract(current_hsv, orange)
        [yellow_mask, yellow_edges] = feature_extract(current_hsv, yellow)
        [blue_mask, blue_edges] = feature_extract(current_hsv, blue)

        # retrieves the rectangular bounding boxes and convex hulls for each colour
        # as well as drawing to the displayed frame
        # from sub module function
        [bounding_box_frame, orange_rects, orange_hulls] = bounds(bounding_box_frame, orange_mask, orange)
        [bounding_box_frame, yellow_rects, yellow_hulls] = bounds(bounding_box_frame, yellow_mask, yellow)
        [bounding_box_frame, orange_rects, orange_hulls] = bounds(bounding_box_frame, blue_mask, blue)


        

        # roi_mask = np.ones(current_frame.shape[:2], dtype="uint8")
        # cv2.rectangle(roi_mask, (yellow_rects[0][0], yellow_rects[0][2]), (yellow_rects[0][1], yellow_rects[0][3]), 0, -1) ,[current_frame.shape[2]-60,500] # mask off sky
        # current_frame = cv2.bitwise_and(current_frame, current_frame, mask=roi_mask)

        # show images
        cv2.imshow("camera-base", current_frame) # image with bounding boxes

        # cv2.imshow("camera-om", orange_mask) # masks for each cone type
        # cv2.imshow("camera-ym", yellow_mask)
        # cv2.imshow("camera-bm", blue_mask)

        # cv2.imshow("camera-oe", orange_edges) # edges for each cone type (used for bounding boxes)
        # cv2.imshow("camera-ye", yellow_edges)
        # cv2.imshow("camera-be", blue_edges)

        # cv2.imshow("camera-o", cv2.bitwise_and(current_frame, current_frame, mask=orange_mask)) # masked frames for each cone type
        # cv2.imshow("camera-y", cv2.bitwise_and(current_frame, current_frame, mask=yellow_mask))
        # cv2.imshow("camera-b", cv2.bitwise_and(current_frame, current_frame, mask=blue_mask))

        # img = np.copy(current_frame)
        # gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        # gray = np.float32(gray)
        # dst = cv2.cornerHarris(gray,2,3,0.04)

        # #result is dilated for marking the corners, not important
        # dst = cv2.dilate(dst,None)

        # # Threshold for an optimal value, it may vary depending on the image.
        # img[dst>0.01*dst.max()]=[0,0,255]

        # cv2.imshow('dst',img)
        
        cv2.waitKey(1)


## main call
def main(args=None):
    rclpy.init(args=args)

    node = Cam_Pipe()
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
