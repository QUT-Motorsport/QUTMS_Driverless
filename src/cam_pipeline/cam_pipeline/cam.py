# ROS2 libraries
import rclpy
from rclpy.node import Node
# ROS2 message libraries
from sensor_msgs.msg import Image
# custom sim data message libraries
from qutms_msgs.msg import ConeScan
# OpenCV2 
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
from pprint import pprint
 
class Cam_Pipe(Node):
    def __init__(self):
        super().__init__('control')

         # creates subscriber to 'cam1' with type Image that sends data to cam_callback
        self.cam_subscription_ = self.create_subscription(
            Image,
            '/fsds/camera/cam1',
            self.cam_callback,
            10)
        self.cam_subscription_  # prevent unused variable warning

        self.br = CvBridge()

    # callback function for camera image processing
    def cam_callback(self, cam_msg):
        # get image
        current_frame =  self.br.imgmsg_to_cv2(cam_msg)
        bounding_box_frame = current_frame # image for drawing bounding boxes on

        # mask off sky and car
        roi_mask = np.ones(current_frame.shape[:2], dtype="uint8")
        cv2.rectangle(roi_mask, (0, 0), (current_frame.shape[1], 400), 0, -1) ,[current_frame.shape[2]-60,500] # mask off sky
        a = 30 # variables for the position of car mask
        b = 100
        c = 235
        h1 = 80
        h2 = 155
        cv2.fillPoly(roi_mask, [np.array([ # massk off car
            [a,current_frame.shape[1]],
            [b, current_frame.shape[1]-h1],
            [c, current_frame.shape[1]-h2],
            [current_frame.shape[0]/2,585],
            [current_frame.shape[0]-c,current_frame.shape[1]-h2],
            [current_frame.shape[0]-b,current_frame.shape[1]-h1],
            [current_frame.shape[0]-a,current_frame.shape[1]]
            ], np.int32)], 0)
        current_frame = cv2.bitwise_and(current_frame, current_frame, mask=roi_mask)

        # convert to hsv
        current_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        # used for removing noise via erode/dilate
        kernel = np.ones((2, 2), np.uint8)

        # mask orange cones
        orange_high = np.array([0.100, 1, 1])*255
        orange_low = np.array([0.013, 0, 0])*255
        orange_mask = cv2.inRange(current_hsv, orange_low, orange_high)
        orange_mask = cv2.dilate(cv2.erode(orange_mask, kernel) , kernel) 

        # mask yellow cones
        yellow_high = np.array([0.2, 1, 1])*255
        yellow_low = np.array([0.1, 0, 0])*255
        yellow_mask = cv2.inRange(current_hsv, yellow_low, yellow_high)
        yellow_mask = cv2.dilate(cv2.erode(yellow_mask, kernel) , kernel) 

        # mask blue cones
        blue_high = np.array([0.6, 1, 1])*255
        blue_low = np.array([0.3, 0.2, 0.5])*255
        blue_mask = cv2.inRange(current_hsv, blue_low, blue_high)
        blue_mask = cv2.dilate(cv2.erode(blue_mask, kernel) , kernel) 

        # find edges around masked areas
        orange_edges = cv2.Canny(orange_mask,100, 200)
        yellow_edges = cv2.Canny(yellow_mask,100, 200)
        blue_edges = cv2.Canny(blue_mask,100, 200)

        # find contours from edges
        orange_contours = cv2.findContours(orange_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]
        yellow_contours = cv2.findContours(yellow_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]
        blue_contours = cv2.findContours(blue_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]

        # find bounding boxes from contours
        orange_rects = list()
        for orange_contour in orange_contours:
            orange_rects.append(cv2.boundingRect(orange_contour))
            x,y,w,h = cv2.boundingRect(orange_contour)
            cv2.rectangle(bounding_box_frame, (x, y), (x+w,y+h),(255,0,0),2)

        yellow_rects = list()
        for yellow_contour in yellow_contours:
            yellow_rects.append(cv2.boundingRect(yellow_contour))
            x,y,w,h = cv2.boundingRect(yellow_contour)
            cv2.rectangle(bounding_box_frame, (x, y), (x+w,y+h),(0,255,0),2)

        blue_rects = list()
        for blue_contour in blue_contours:
            x,y,w,h = cv2.boundingRect(blue_contour)
            blue_rects.append(cv2.boundingRect(blue_contour))
            cv2.rectangle(bounding_box_frame, (x, y), (x+w,y+h),(0,0,255),2)
        
        # show images
        cv2.imshow("camera-base", bounding_box_frame) # image with bounding boxes

        # cv2.imshow("camera-om", orange_mask) # masks for each cone type
        # cv2.imshow("camera-ym", yellow_mask)
        # cv2.imshow("camera-bm", blue_mask)

        # cv2.imshow("camera-oe", orange_edges) # edges for each cone type (used for bounding boxes)
        # cv2.imshow("camera-ye", yellow_edges)
        # cv2.imshow("camera-be", blue_edges)

        # cv2.imshow("camera-o", cv2.bitwise_and(current_frame, current_frame, mask=orange_mask)) # masked frames for each cone type
        # cv2.imshow("camera-y", cv2.bitwise_and(current_frame, current_frame, mask=yellow_mask))
        # cv2.imshow("camera-b", cv2.bitwise_and(current_frame, current_frame, mask=blue_mask))
        
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
