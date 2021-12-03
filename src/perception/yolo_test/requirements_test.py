## Importing libraries ##
import cv2
import numpy as np
import torch
import time

from typing import Generator, List, Tuple
from rect import Rect


YELLOW_DISP_COLOUR: tuple = (0, 255, 255)  # bgr - yellow
BLUE_DISP_COLOUR: tuple = (255, 0, 0)  # bgr - blue
ORANGE_DISP_COLOUR: tuple = (0, 165, 255)  # bgr - orange

YOLO_CONE_DETECTION_PARAMETERS = [
    (0, BLUE_DISP_COLOUR),
    (1, YELLOW_DISP_COLOUR),
    (2, ORANGE_DISP_COLOUR),
]

CLASS = 5

#### initialising function for the yolov5 model ####
def yolov5_init(threshold):

    #loading the model
    model = torch.hub.load('ultralytics/yolov5', 'custom', path="src/perception/yolo_test/bestOne.pt") 
    model.conf = threshold
    return model


# initiating yolov5 model with a threshold of 0.4
model = yolov5_init(0.45)

cv_image = cv2.imread("src/perception/yolo_test/testImage.jpg")
rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB) # make to RGB image

#inference 
results = model(rgb_image)

# rendering and getting image RGB array
im_array = results.render()[0]
bgr_image = cv2.cvtColor(im_array, cv2.COLOR_BGR2RGB) # make to RGB image

# getting the bounding boxes of all the detected cones: 
data = results.pandas().xyxy[0]

bounding_boxes = []
for cone_colour, display_colour in YOLO_CONE_DETECTION_PARAMETERS:
    for i in range(len(data.index)): 
        if data.iloc[i, CLASS] == cone_colour:
            bounding_box = Rect(data.xmin[i], data.ymin[i], (data.xmax[i]-data.xmin[i]), (data.ymax[i]-data.ymin[i]))
            bounding_boxes.append(bounding_box)

cv2.imshow('image', bgr_image)

cv2.waitKey(1)

time.sleep(30)