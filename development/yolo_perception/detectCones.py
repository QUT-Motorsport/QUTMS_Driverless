# QUT Motorsport - Driverless

# A node that utilises yolov5 to detect cones and provide guidance solutions

# Author: Lachlan Masson

## Importing libraries ##
from __future__ import print_function
import sys
import rclpy
import cv2
import os
import torch
import execnet
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#############################
##### utility functions #####
#############################

#### initialising function for the yolov5 model ####
def yolov5_init(threshold):

    #loading the model
    model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/lachie/Desktop/yoloWeights/bestOne.pt') 
    model.conf = threshold
    return model

#### a function which gets the centre of a bounding box (rectangle in this instance) ####
def getCentroid(x1, y1, x2, y2):

    xC = (x1 + x2) / 2 
    yC = (y1 + y2) / 2

    xCyC = []
    xCyC.append(xC)
    xCyC.append(yC)

    return xCyC

#### a function for the main yolov5 detector ####
def yolov5_detector(model, image):

    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    #inference 
    results = model(image)

    #rendering and getting image list
    im_list = results.render()
    im_list = [x[..., ::-1] for x in im_list]  #RGB to BGR


    # getting the bounding boxes of all the detected cones: 
    #boundingBoxes = results.xyxy[0]
    #print('\n', boundingBoxes)

    # number of cones detected: 
    numConesDetected = len(results.xyxy[0])
    print('the number of cones is: ', numConesDetected)

    # an array that stores all of the blue traffic cones detected: 
    blueCones = []
    for i in range(0, numConesDetected):
        if(results.xyxy[0][i][5] == 0):
            blueCones.append(results.xyxy[0][i])

    print('the number of blue cones is: ',len(blueCones))


    # an array that stores all of the yellow traffic cones detected: 
    yellowCones = []
    for i in range(0, numConesDetected):
        if(results.xyxy[0][i][5] == 1):
            yellowCones.append(results.xyxy[0][i])

    print('the number of yellow cones is: ', len(yellowCones))


    # calculating the centroids of the blue cones:
    blueConeCentroids = []
    for i in range(0, len(blueCones)):
        blueConeCentroids.append(getCentroid(blueCones[i][0], blueCones[i][1], blueCones[i][2], blueCones[i][3]))

    print('Blue cone centroids', blueConeCentroids)


    # calculating the centroids of the yellow cones: 
    yellowConeCentroids = []
    for i in range(0, len(yellowCones)):
        yellowConeCentroids.append(getCentroid(yellowCones[i][0], yellowCones[i][1], yellowCones[i][2], yellowCones[i][3]))

    print('Yellow cone centroids: ', yellowConeCentroids)

    # getting the closet yellow cone (based on y position): 
    closetBlue = []

    if(len(blueConeCentroids) > 0):

        minBlue = blueConeCentroids[0][1]
        closetBlue = blueConeCentroids[0]

        for i in range(0, len(blueConeCentroids) ):
            if(blueConeCentroids[i][1] > minBlue):
                closetBlue = blueConeCentroids[i]


    print('The closet blue cone is located at: ', closetBlue)

    # getting the closet blue cone (based on y position)
    closetYellow = []

    if(len(yellowConeCentroids) > 0):

        minYellow = yellowConeCentroids[0][1]
        closetYellow = yellowConeCentroids[0]

        for i in range(0, len(yellowConeCentroids) ):
        if(yellowConeCentroids[i][1] > minYellow):
            closetYellow = yellowConeCentroids[i]

    print('The closet yellow cone is located at: ', closetYellow)


    # calculating the midpoint of the closet blue and yellow cones:
    targetXY= []
    targetXY = getCentroid(closetBlue[0], closetBlue[1], closetYellow[0], closetYellow[1])

    print('The target coordinates are: ', targetXY)

    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    targetImage = cv2.circle(image, (236, 322), 10, (0,0,255), -1)

    blueConeImage = cv2.circle(image, (59, 326), 10, (255,0,0), -1)

    yellowConeImage = cv2.circle(image, (413, 318), 10, (0,255,255), -1)



    cv2.imshow('target', targetImage)
    #cv2.imshow('blue', blueConeImage)
    #cv2.imshow('yellow', yellowConeImage)

    #showing the image 
    #cv2.imshow('image',im_list[0])



# initiating yolov5 model with a threshold of 0.4
model = yolov5_init(0.45)


class MinimalSubscriber(Node):

    # Actual topic  - 'fsds/camera/cam1' 
    # Testing topic - 'Image'

    def __init__(self):

        super().__init__('minimal_subscriber')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(Image, 'Image', self.listener_callback, 10)
        
        self.subscription


    def listener_callback(self, msg):
        try:

        #original = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        detected = self.bridge.imgmsg_to_cv2(msg, "bgr8")


        except CvBridgeError as e:
        print(e)


        print('Image detected!, performing yolov5 object detection...\n')

        yolov5_detector(model, detected)
    
        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()