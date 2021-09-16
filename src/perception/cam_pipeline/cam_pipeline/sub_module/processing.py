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

FOCAL_LEN = 1
SMALL_CONE_W = 0.228
SMALL_CONE_H = 0.505
LARGE_CONE_W = 0.228
LARGE_CONE_H = 0.325

def distance_calc(rect, colour):
    centroid = [rect[0]+rect[2]/2, rect[1]+rect[3]/2] # [x,y] pixel coords
    height = abs(rect[3] - rect[1]) # height of box
    width = abs(rect[2] - rect[0]) # width of box

    # work out which cone is which
    if colour != "orange":
        width_ratio = width * SMALL_CONE_W
        z = height * SMALL_CONE_H
    else:
        width_ratio = width * LARGE_CONE_W
        z = height * LARGE_CONE_H

    x = (SMALL_CONE_W * FOCAL_LEN) / width
    yoffset = centroid[1]
    y = width_ratio * yoffset

    return [x, y, z]


def feature_extract(current_hsv, colour):
    # mask orange cones
    mask = cv2.inRange(current_hsv, lower_thresh[colour], upper_thresh[colour])
    mask = cv2.dilate(cv2.erode(mask, kernel) , kernel) 

    edges = cv2.Canny(mask, 100, 200)
    
    return [mask, edges]


def bounds(bounding_box_frame, mask, colour): # rects are returned as (x1, y1, x2, y2) not (x, y, w, h)
    # find contours from edges
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]
    box_min_size = 2

    # find bounding boxes from contours
    rects = list()
    for i in range(len(contours)):
        [x,y,w,h] = cv2.boundingRect(contours[i])

        # box_min_size = 3
        # if w>box_min_size and h>box_min_size:
        rects.append([x,y,w,h])

        # draw rectangles + centroids for 
        cv2.rectangle(bounding_box_frame, (x, y), (x+w,y+h),(255,0,0),2)
        # cv2.circle(bounding_box_frame, (x+w//2, y+h//2), 1, (255, 0, 255), 2)

    rects.sort(reverse=True, key=lambda rect: rect[2]*rect[3])
    new_rects = list() # create new list of bounding rectangles for whole cone (not segments)
    coords = list()
    while len(rects) > 0: # loop until empty
        rect = rects.pop(0)
        rect = [rect[0], rect[1], rect[0]+rect[2], rect[1]+rect[3]]
        i = 0
        while i < len(rects):
            other_rect = rects[i]
            if other_rect[0] > rect[0] and other_rect[0]+other_rect[2] < rect[2]:
                rect[0] = min(rect[0], other_rect[0])
                rect[1] = min(rect[1], other_rect[1])
                rect[2] = max(rect[2], other_rect[0]+other_rect[2])
                rect[3] = max(rect[3], other_rect[1]+other_rect[3])
                rects.remove(other_rect)
            else:
                i += 1
        new_rects.append(rect)

        # send rectangle for distance calculation
        coord = distance_calc(rect)
        coords.append(coord) # append to coord list

        cv2.rectangle(bounding_box_frame, tuple(rect[0:2]), tuple(rect[2:]),(0,0,255),2)

    return [bounding_box_frame, coords]    