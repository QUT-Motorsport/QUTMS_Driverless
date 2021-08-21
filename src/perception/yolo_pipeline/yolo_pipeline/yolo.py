# QUT Motorsport - Driverless
# A node that utilises yolov5 to detect cones and provide guidance solutions
# Author: Lachlan Masson

# ROS2 libraries
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
# ROS2 message libraries
from sensor_msgs.msg import Image
# custom sim data message libraries
from qutms_msgs.msg import ConeScan, ConeData
# other python libraries
import torch
import cv2

#############################
##### utility functions #####
#############################

## a function which gets the centre of a bounding box (rectangle in this instance)
def get_centroid(x1, y1, x2, y2):
    xC = (x1 + x2) / 2 
    yC = (y1 + y2) / 2

    xCyC = []
    xCyC.append(xC)
    xCyC.append(yC)

    return xCyC

## a function for the main yolov5 detector
def yolov5_detector(model, image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    #inference 
    results = model(image)

    #rendering and getting image list
    im_list = results.render()
    im_list = [x[..., ::-1] for x in im_list]  #RGB to BGR


    # getting the bounding boxes of all the detected cones: 
    # boundingBoxes = results.xyxy[0]
    # print('\n', boundingBoxes)

    # number of cones detected: 
    numConesDetected = len(results.xyxy[0])
    # print('the number of cones is: ', numConesDetected)


    # an array that stores all of the blue traffic cones detected: 
    blueCones = []
    for i in range(0, numConesDetected):
        if(results.xyxy[0][i][5] == 0):
            blueCones.append(results.xyxy[0][i])
    # print('the number of blue cones is: ',len(blueCones))

    # an array that stores all of the yellow traffic cones detected: 
    yellowCones = []
    for i in range(0, numConesDetected):
        if(results.xyxy[0][i][5] == 1):
            yellowCones.append(results.xyxy[0][i])
    # print('the number of yellow cones is: ', len(yellowCones))


    # calculating the centroids of the blue cones:
    blueConeCentroids = []
    for i in range(0, len(blueCones)):
        blueConeCentroids.append(get_centroid(blueCones[i][0], blueCones[i][1], blueCones[i][2], blueCones[i][3]))
    # print('Blue cone centroids', blueConeCentroids)

    # calculating the centroids of the yellow cones: 
    yellowConeCentroids = []
    for i in range(0, len(yellowCones)):
        yellowConeCentroids.append(get_centroid(yellowCones[i][0], yellowCones[i][1], yellowCones[i][2], yellowCones[i][3]))
    # print('Yellow cone centroids: ', yellowConeCentroids)


    # getting the closet yellow cone (based on y position): 
    closetBlue = []
    if(len(blueConeCentroids) > 0):
        minBlue = blueConeCentroids[0][1]
        closetBlue = blueConeCentroids[0]

        for i in range(0, len(blueConeCentroids) ):
            if(blueConeCentroids[i][1] > minBlue):
                closetBlue = blueConeCentroids[i]
    # print('The closet blue cone is located at: ', closetBlue)

    # getting the closet blue cone (based on y position)
    closetYellow = []
    if(len(yellowConeCentroids) > 0):
        minYellow = yellowConeCentroids[0][1]
        closetYellow = yellowConeCentroids[0]

        for i in range(0, len(yellowConeCentroids) ):
            if(yellowConeCentroids[i][1] > minYellow):
                closetYellow = yellowConeCentroids[i]
    # print('The closet yellow cone is located at: ', closetYellow)


    # calculating the midpoint of the closet blue and yellow cones:
    targetXY= []
    targetXY = get_centroid(closetBlue[0], closetBlue[1], closetYellow[0], closetYellow[1])
    # print('The target coordinates are: ', targetXY)

    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    # targetImage = cv2.circle(image, (236, 322), 10, (0,0,255), -1)
    # blueConeImage = cv2.circle(image, (59, 326), 10, (255,0,0), -1)
    # yellowConeImage = cv2.circle(image, (413, 318), 10, (0,255,255), -1)


    # cv2.imshow('target', targetImage)
    # cv2.imshow('blue', blueConeImage)
    # cv2.imshow('yellow', yellowConeImage)

    # showing the image 
    return image 


## initialising function for the YOLOv5 model
def yolov5_init(threshold):
    #loading the model
    model = torch.hub.load(
        'ultralytics/yolov5', 
        'custom', 
        path='/home/developer/driverless_ws/src/yolo_pipeline/bestOne.pt') # location of model in docker env
    model.conf = threshold
    return model


class YOLODetection(Node):
    def __init__(self):
        super().__init__('YOLO_detection')
        ## creates subscriber to 'cam1' with type Image that sends data to cam_callback     
        self.cam_subscription_ = self.create_subscription(
            Image, 
            # 'Image', # Testing topic - 'Image'
            'fsds/camera/cam1', # Actual topic  - 'fsds/camera/cam1' 
            self.cam_callback, 
            10)
        self.cam_subscription_  # prevent unused variable warning
        self.bridge = CvBridge() # define OpenCV bridge to AirSim environment
        # initiating yolov5 model with a threshold of 0.45
        self.model = yolov5_init(0.45)
        self.display = False

        ## creates publisher to 'lidar_output' with type ConeScan
        self.frame_publisher_ = self.create_publisher(
            ConeScan,
            'cam_output', 
            10)
        # creates timer for publishing commands
        self.timer_period = 0.001  # seconds
        self.timer = self.create_timer(self.timer_period, self.publisher)


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

    
    def publisher(self):
        cone_scan = ConeScan()
        # head = Header()

        cone_data = []
        for i in range(len(self.cones)):
            cone = ConeData()
            cone.x = self.cones[i][0]
            cone.y = self.cones[i][1]
            cone.z = self.cones[i][2]
            cone.i = self.cones[i][3]
            cone_data.append(cone)
       
        # head.stamp = rospy.Time.now()
        # head.frame_id = "lidar2"
        # cone_scan.header = head

        cone_scan.data = cone_data

        self.scan_publisher_.publish(cone_scan)


def main(args=None):
    rclpy.init(args=args)

    node = YOLODetection()
    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()