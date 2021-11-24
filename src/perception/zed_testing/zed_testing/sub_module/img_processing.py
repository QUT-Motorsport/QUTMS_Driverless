# import python libraries
import cv2
import numpy as np

# HSV limits
orange_low = np.array([0, 100, 50])
orange_high = np.array([15, 255, 255])

# used for removing noise via erode/dilate
kernel = np.ones((3, 3), np.uint8)

min_box_sizes = 3 # pixels

def feature_extract(gauss_frame):
    # mask orange cones
    mask = cv2.inRange(gauss_frame, orange_low, orange_high)
    corrected = cv2.erode(cv2.dilate(mask, kernel, iterations=1) , kernel, iterations=1)

    return corrected


def bounds(bounding_box_frame, cones, mask):
    # find contours from edges
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # find bounding boxes from contours
    rects = list() # create blank list of rectangles
    for i in range(len(contours)):
        [x,y,w,h] = cv2.boundingRect(contours[i])

        # box size cutoff
        min_pix_size = 0 # can change based on sim image
        if w > min_pix_size and h > min_pix_size:
            # append rectangle geometry twice to ensure rectangle groupings will 
            # show initially non-overlapping rectangles
            rects.append([x,y,x+w,y+h])

        # draw rectangles for segments (green rectangle to distinguish)
        # cv2.rectangle(bounding_box_frame, (x, y), (x+w, y+h), (0, 204, 0), 2)
    
    # for r in results:
    for r in rects:
        left_edge = r[0] # left of rect at initial x coord (top left of rect)
        right_edge = r[2] # right at final x coord
        top_edge = r[1] # top at initial y coord
        bottom_edge = r[3] # bottom at final y coord
        
        width = right_edge - left_edge # width and hight in pixels
        height = bottom_edge - top_edge
        x_centroid = int(left_edge + width / 2) # centre of rectangle
        y_centroid = int(top_edge + height / 2)

        # only take whole objects, not false positives
        if width>min_box_sizes and height>min_box_sizes:
            # draw rectangle with object geometry
            cv2.rectangle(
                bounding_box_frame, # image to draw on
                (left_edge, top_edge), (right_edge, bottom_edge), # geometry to draw between
                (0, 255, 0), 1) # colour and thickness of box (pixels)

            # append object properties to array
            cones.append([x_centroid, y_centroid, left_edge, top_edge, width, height])

    return [bounding_box_frame, cones] # return bounds for each object + drawn-on frame


def cam_main(frame):
    # used to crop frame to only required area
    h, w, _ = frame.shape

    left_frame = frame[0:h, 0:int(w/2-1)]
    right_frame = frame[0:h, int(w/2):w]

    bounding_box_frame = np.copy(left_frame) # frame to display on

    # convert to hsv
    current_hsv = cv2.cvtColor(left_frame, cv2.COLOR_BGR2HSV)
    gauss_frame = cv2.GaussianBlur(current_hsv, (3,3), cv2.BORDER_DEFAULT)

    cones = list() # init blank list of cones
    # gets updated each time a new cone is found

    # retrieves the mask and edges for each colour from sub module function
    corrected = feature_extract(gauss_frame)

    # retrieves the rectangular bounding boxes for each colour as well as 
    # drawing to the displayed frame from sub module function
    [bounding_box_frame, cones] = bounds(bounding_box_frame, cones, corrected)

    # for cone in cones:
    #     intensity = depth_frame[cone[1], cone[0]]
            
    #     string = "depth: " + str(intensity)
    #     cv2.putText(
    #         bounding_box_frame, string, # frame to draw on, label to draw
    #         (cone[2], cone[3]-5), # position
    #         cv2.FONT_HERSHEY_SIMPLEX, 0.4, # font, font size
    #         (0, 255, 0), 1) # red font colour, thickness 1

    return [cones, bounding_box_frame, left_frame]