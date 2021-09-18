# import python libraries
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
col_words = ["blue", "yellow", "orange"]
box_cols = [(0, 102, 255), (0, 255, 255), (255, 102, 0)]

# used for removing noise via erode/dilate
kernel = np.ones((2, 2), np.uint8)


def feature_extract(current_hsv, colour):
    # mask orange cones
    mask = cv2.inRange(current_hsv, lower_thresh[colour], upper_thresh[colour])
    mask = cv2.dilate(cv2.erode(mask, kernel) , kernel) 

    edges = cv2.Canny(mask, 100, 200)
    
    return [mask, edges]


def bounds(bounding_box_frame, cones, mask, colour): # rects are returned as (x1, y1, x2, y2) not (x, y, w, h)
    # find contours from edges
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]

    # find bounding boxes from contours
    rects = list() # create blank list of rectangles
    for i in range(len(contours)):
        [x,y,w,h] = cv2.boundingRect(contours[i])

        # box size cutoff
        min_pix_size = 3 # can change based on sim image
        if w > min_pix_size and h > min_pix_size:
            rects.append([x,y,w,h]) # add rectangle geometry to list

        # draw rectangles for segments (green rectangle to distinguish)
        # cv2.rectangle(bounding_box_frame, (x, y), (x+w, y+h), (0, 204, 0), 2)

    # find whole cones' bounding rectangles
    rects.sort(reverse=True, key=lambda rect: rect[2]*rect[3])

    while len(rects) > 0: # loop until empty
        rect = rects.pop(0) # remove element 0 from rects and return it
        rect = [rect[0], rect[1], rect[0]+rect[2], rect[1]+rect[3]] # x1, y1, x2, y2
        
        left_edge = rect[0]
        top_edge = rect[1]
        right_edge = rect[2]
        bottom_edge = rect[3]

        i = 0
        while i < len(rects):
            other_rect = rects[i]
            if other_rect[0] > rect[0] and other_rect[0]+other_rect[2] < rect[2]:
                left_edge = min(rect[0], other_rect[0])
                top_edge = min(rect[1], other_rect[1])
                right_edge = max(rect[2], other_rect[0]+other_rect[2])
                bottom_edge = max(rect[3], other_rect[1]+other_rect[3])
                rects.remove(other_rect)
            else:
                i += 1

        width = right_edge - left_edge # width and hight in pixels
        height = bottom_edge - top_edge
        x_centroid = (left_edge + width / 2) # centre of rectangle
        y_centroid = (top_edge + height / 2)
        
        # append cone properties to array
        cones.append([col_words[colour], width, height, x_centroid, y_centroid, left_edge, top_edge])

        # draw rectangle with cone geometry
        cv2.rectangle(
            bounding_box_frame, # image to draw on
            (left_edge, top_edge), (right_edge, bottom_edge), # geometry to draw between
            box_cols[colour], 2) # colour and thickness of box (pixels)

        # draw cone centroid dot
        # cv2.circle(bounding_box_frame, (x+w//2, y+h//2), 1, (255, 0, 255), 2)
        
    return [bounding_box_frame, cones]


def cam_main(frame, display):
    # used to crop frame to only required area
    h, w, _ = frame.shape

    # for OG settings script
    if h == 785 and w == 785:
        # mask off sky and car
        roi_mask = np.ones(frame.shape[:2], dtype="uint8")
        cv2.rectangle(roi_mask, (0, 0), (frame.shape[1], 400), 0, -1) ,[frame.shape[2]-60,500] # mask off sky
        a = 30 # variables for the position of car mask
        b = 100
        c = 235
        h1 = 80
        h2 = 155
        # mask off car
        cv2.fillPoly(roi_mask, [np.array([
            [a,frame.shape[1]],
            [b, frame.shape[1]-h1],
            [c, frame.shape[1]-h2],
            [frame.shape[0]/2,585],
            [frame.shape[0]-c,frame.shape[1]-h2],
            [frame.shape[0]-b,frame.shape[1]-h1],
            [frame.shape[0]-a,frame.shape[1]]
            ], np.int32)], 0)
        
        current_frame = cv2.bitwise_and(frame, frame, mask=roi_mask)

    # for higher res (variable masking)
    else:
        # vertical res: halfway down bottom
        # horizontal res: full
        current_frame = frame[int(16*h/30):h, 0:w]# made variable to cam resolution

    bounding_box_frame = current_frame # frame to display on

    # convert to hsv
    current_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

    cones = list() # init blank list of cones
    # gets updated each time a new cone is found

    # passable variables for colours
    #   blue = 0
    #   yellow = 1
    #   orange = 2
    for colour in range(0, len(colours)):
        # retrieves the mask and edges for each colour from sub module function
        [mask, edges] = feature_extract(current_hsv, colour)

        # retrieves the rectangular bounding boxes for each colour as well as 
        # drawing to the displayed frame from sub module function
        [bounding_box_frame, cones] = bounds(bounding_box_frame, cones, mask, colour)


    # roi_mask = np.ones(current_frame.shape[:2], dtype="uint8")
    # cv2.rectangle(roi_mask, (yellow_rects[0][0], yellow_rects[0][2]), (yellow_rects[0][1], yellow_rects[0][3]), 0, -1) ,[current_frame.shape[2]-60,500] # mask off sky
    # current_frame = cv2.bitwise_and(current_frame, current_frame, mask=roi_mask)

    if display == True:
        # show bounding boxes
        cv2.imshow("camera-base", bounding_box_frame) # image with bounding boxes

        # img = np.copy(current_frame)
        # gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        # gray = np.float32(gray)
        # dst = cv2.cornerHarris(gray,2,3,0.04)

        # #result is dilated for marking the corners, not important
        # dst = cv2.dilate(dst,None)

        # # Threshold for an optimal value, it may vary depending on the image.
        # img[dst>0.01*dst.max()]=[0,0,255]

        # cv2.imshow('dst',img)

        cv2.waitKey(1)

    return cones