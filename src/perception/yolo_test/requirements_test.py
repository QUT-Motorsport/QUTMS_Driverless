## Importing libraries ##
import cv2
import numpy as np
import torch
import time


#### initialising function for the yolov5 model ####
def yolov5_init(threshold):

    #loading the model
    model = torch.hub.load('ultralytics/yolov5', 'custom', path="/home/developer/driverless_ws/src/perception/yolo_test/bestOne.pt") 
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

    # rendering and getting image list
    im_list = results.render()
    print(im_list)
    im_list = [x[..., ::-1] for x in im_list]  # RGB to BGR

    # getting the bounding boxes of all the detected cones: 
    boundingBoxes = results.pandas().xyxy[0]
    print('\n', boundingBoxes)

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

    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    cv2.imshow('image',im_list[0])

    cv2.waitKey(1)

    time.sleep(15)


# initiating yolov5 model with a threshold of 0.4
model = yolov5_init(0.45)
cv_image = cv2.imread("/home/developer/driverless_ws/src/perception/yolo_test/testImage.jpg")

yolov5_detector(model, cv_image)

