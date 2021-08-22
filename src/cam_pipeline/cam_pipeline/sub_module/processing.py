# import the necessary packages
import cv2
import numpy as np

# HSV limits
orange_high = np.array([0.100, 1, 1])*255
orange_low = np.array([0.013, 0, 0])*255

yellow_high = np.array([0.2, 1, 1])*255
yellow_low = np.array([0.1, 0, 0])*255

blue_high = np.array([0.6, 1, 1])*255
blue_low = np.array([0.3, 0.2, 0.5])*255

upper_thresh = [orange_high, yellow_high, blue_high]
lower_thresh = [orange_low, yellow_low, blue_low]

colours = [0, 1, 2]
col_values = [(255,0,0), (0,255,0), (0,0,255)]

# used for removing noise via erode/dilate
kernel = np.ones((2, 2), np.uint8)


def feature_extract(current_hsv, colour):
    # mask orange cones
    mask = cv2.inRange(current_hsv, lower_thresh[colour], upper_thresh[colour])
    mask = cv2.dilate(cv2.erode(mask, kernel) , kernel) 

    edges = cv2.Canny(mask, 100, 200)
    
    return [mask, edges]


def bounds(bounding_box_frame, mask, colour):
    # find contours from edges
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]

    box_min_size = 2

    # find bounding boxes from contours
    rects = []
    hulls = []
    for i in range(len(contours)):
        [x,y,w,h] = cv2.boundingRect(contours[i])
        hull = cv2.convexHull(contours[i])
        hulls.append(hull)
        if w>box_min_size and h>box_min_size:
            # cv2.rectangle(bounding_box_frame, (x, y), (x+w,y+h),(255,0,0),2)
            rects.append([x,y,w,h])
        cv2.drawContours(bounding_box_frame, hulls, i, col_values[colour], 2)

    return [bounding_box_frame, rects, hulls]