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
 
class Cam_Pipe(Node):
    def __init__(self):
        super().__init__('control')

         # creates subscriber to 'cam1' with type PointCloud2 that sends data to lidar_callback
        self.cam_subscription_ = self.create_subscription(
            Image,
            '/fsds/camera/cam1',
            self.cam_callback,
            10)
        self.cam_subscription_  # prevent unused variable warning

        self.br = CvBridge()

    
    def cam_callback(self, cam_msg):
        # get image and convert to hsv
        current_frame =  self.br.imgmsg_to_cv2(cam_msg)
        current_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        # mask orange cones
        orange_high = np.array([0.100, 1, 1])*255
        orange_low = np.array([0.013, 0, 0])*255
        orange_mask = cv2.inRange(current_hsv, orange_low, orange_high)

        # mask yellow cones
        yellow_high = np.array([0.2, 1, 1])*255
        yellow_low = np.array([0.1, 0, 0])*255
        yellow_mask = cv2.inRange(current_hsv, yellow_low, yellow_high)

        # mask blue cones
        blue_high = np.array([0.6, 1, 1])*255
        blue_low = np.array([0.3, 0.2, 0.5])*255
        blue_mask = cv2.inRange(current_hsv, blue_low, blue_high)

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

        # show images
        cv2.imshow("camera-o", cv2.bitwise_and(current_frame, current_frame, mask=orange_mask))
        cv2.imshow("camera-y", cv2.bitwise_and(current_frame, current_frame, mask=yellow_mask))
        cv2.imshow("camera-b", cv2.bitwise_and(current_frame, current_frame, mask=blue_mask))
        
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
