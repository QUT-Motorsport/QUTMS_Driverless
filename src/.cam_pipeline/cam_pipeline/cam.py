# OpenCV2 
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
 
class Controller(Node):
    def __init__(self):
        super().__init__('control')

 
        ## creates subscriber to 'cam1' with type PointCloud2 that sends data to lidar_callback
        # self.cam_subscription_ = self.create_subscription(
        #     Image,
        #     '/fsds/camera/cam1',
        #     self.cam_callback,
        #     10)
        # self.cam_subscription_  # prevent unused variable warning

