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


def bounds(bounding_box_frame, mask, colour): # rects are returned as (x1, y1, x2, y2) not (x, y, w, h)
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
        # if w>box_min_size and h>box_min_size:
        cv2.rectangle(bounding_box_frame, (x, y), (x+w,y+h),(255,0,0),2)
        rects.append([x,y,w,h])
        # cv2.drawContours(bounding_box_frame, hulls, i, col_values[colour], 2)
        # cv2.circle(bounding_box_frame, (x+w//2, y+h//2), 1, (255, 0, 255), 2)

    rects.sort(reverse=True, key=lambda rect: rect[2]*rect[3])
    new_rects = list()
    while len(rects) > 0:
        rect = rects.pop(0)
        centroid = [rect[0]+rect[2]/2, rect[1]+rect[3]/2]
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
        cv2.rectangle(bounding_box_frame, tuple(rect[0:2]), tuple(rect[2:]),(0,0,255),2)

    return [bounding_box_frame, new_rects, hulls]    