# QUT Motorsport - Driverless
# A node that utilises yolov5 to detect cones and provide guidance solutions
# Author: Lachlan Masson

# import ROS2 libraries
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
# import ROS2 message libraries
from sensor_msgs.msg import Image
# custom sim data message libraries
from qutms_msgs.msg import ConeScan, ConeData

# import other python libraries
import torch
import cv2

# import helper image processing module
from .sub_module.yolo_processing import *


## initialising function for the YOLOv5 model
def yolov5_init(threshold):
    #loading the model
    model = torch.hub.load(
        'ultralytics/yolov5', 
        'custom', 
        path='/home/developer/driverless_ws/src/perception/yolo_pipeline/bestOne.pt') # location of model in docker env
    model.conf = threshold
    return model


class YOLODetection(Node):
    def __init__(self):
        super().__init__('YOLO_detection')
        ## creates subscriber to 'cam2' with type Image that sends data to cam_callback     
        self.cam_subscription_ = self.create_subscription(
            Image, 
            # '/fsds/camera/cam1', # 785x785 (square defaults)
            '/fsds/camera/cam2', # 1080p (like a usual camera)
            self.cam_callback, 
            10)
        self.cam_subscription_  # prevent unused variable warning
        self.bridge = CvBridge() # define OpenCV bridge to AirSim environment
        # initiating yolov5 model with a threshold of 0.45
        self.model = yolov5_init(0.45)
        self.display = True

        ## creates publisher to 'lidar_output' with type ConeScan
        # self.frame_publisher_ = self.create_publisher(
        #     ConeScan,
        #     'cam_output', 
        #     10)
        # # creates timer for publishing commands
        # self.timer_period = 0.001  # seconds
        # self.timer = self.create_timer(self.timer_period, self.publisher)


    def cam_callback(self, msg):
        try:
            #original = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        except CvBridgeError as e:
            print(e)

        h, w, _ = frame.shape
        cropped = frame[int(h/2):int(h*3/4), 0:w]

        # print('Image detected!, performing yolov5 object detection...\n')
        image = yolov5_detector(self.model, cropped)
        
        if self.display == True:
            # cv2.imshow("camera", frame)
            cv2.imshow('image', image)

        cv2.waitKey(1)

    
    # def publisher(self):
    #     cone_scan = ConeScan()
    #     # head = Header()

    #     cone_data = []
    #     for i in range(len(self.cones)):
    #         cone = ConeData()
    #         cone.x = self.cones[i][0]
    #         cone.y = self.cones[i][1]
    #         cone.z = self.cones[i][2]
    #         cone.i = self.cones[i][3]
    #         cone_data.append(cone)
       
    #     # head.stamp = rospy.Time.now()
    #     # head.frame_id = "lidar2"
    #     # cone_scan.header = head

    #     cone_scan.data = cone_data

    #     self.scan_publisher_.publish(cone_scan)


def main(args=None):
    rclpy.init(args=args)

    node = YOLODetection()
    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()