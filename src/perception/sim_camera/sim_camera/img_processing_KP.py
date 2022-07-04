# import python libraries
import os
import random

import cv2
import numpy as np

# HSV limits
orange_high = np.array([0.100, 1, 1]) * 255
orange_low = np.array([0.013, 0, 0]) * 255

yellow_high = np.array([0.2, 1, 1]) * 255
yellow_low = np.array([0.1, 0, 0]) * 255

blue_high = np.array([0.6, 1, 1]) * 255
blue_low = np.array([0.3, 0.2, 0.5]) * 255

upper_thresh = [blue_high, yellow_high, orange_high]
lower_thresh = [blue_low, yellow_low, orange_low]

colours = [0, 1, 2]
col_words = ["blue", "yellow", "orange"]
box_cols = [(255, 102, 0), (0, 255, 255), (0, 102, 255)]

max_cone_aspect = [1.5, 1.5, 1.7]  # blue, yellow orange - blue and yellow should probably be the same
min_cone_aspect = [1.1, 1.1, 1.5]

# used for removing noise via erode/dilate
kernel = np.ones((2, 2), np.uint8)


def feature_extract(current_hsv, colour):
    # mask orange cones
    mask = cv2.inRange(current_hsv, lower_thresh[colour], upper_thresh[colour])
    mask = cv2.dilate(cv2.erode(mask, kernel), kernel)

    edges = cv2.Canny(mask, 100, 200)

    return [mask, edges]


def bounds(bounding_box_frame, cones, mask, colour):
    # find contours from edges
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]

    # find bounding boxes from contours
    rects = list()  # create blank list of rectangles
    for i in range(len(contours)):
        [x, y, w, h] = cv2.boundingRect(contours[i])

        # box size cutoff
        min_pix_size = 0  # can change based on sim image
        if w > min_pix_size and h > min_pix_size:
            # append rectangle geometry twice to ensure rectangle groupings will
            # show initially non-overlapping rectangles
            rects.append([x, y, w, h])

        # draw rectangles for segments (green rectangle to distinguish)
        # cv2.rectangle(bounding_box_frame, (x, y), (x+w, y+h), (0, 204, 0), 2)

    # find whole cones' bounding rectangles
    rects.sort(reverse=True, key=lambda rect: rect[2] * rect[3])

    # loop removes all boundng boxes tounching the edge of the frame
    i = 0
    h, w, _ = bounding_box_frame.shape
    while i < len(rects):
        rect = rects[i]
        rect = [rect[0], rect[1], rect[0] + rect[2], rect[1] + rect[3]]  # left, top, right, bottom

        if min(rect[0:2]) <= 0 or rect[2] >= w or rect[3] >= h:
            rects.pop(i)
        else:
            i += 1

    # loop merges multiple bounding boxes, idealy over one cone
    while len(rects) > 0:  # loop until empty
        rect = rects.pop(0)  # remove element 0 (next biggest bounding box) from results and return it
        rect = [rect[0], rect[1], rect[0] + rect[2], rect[1] + rect[3]]  # left, top, right, bottom

        left_edge = rect[0]
        top_edge = rect[1]
        right_edge = rect[2]
        bottom_edge = rect[3]

        # loop checks each smaller bounding box, merging matching boxes into rect
        i = 0
        while i < len(rects):
            other_rect = rects[i]  # get next smallest box
            if (
                other_rect[0] > rect[0] and other_rect[0] + other_rect[2] < rect[2]
            ):  # check if smaller box is within larger box horizontaly
                # get new edges
                left_edge1 = min(rect[0], other_rect[0])
                top_edge1 = min(rect[1], other_rect[1])
                right_edge1 = max(rect[2], other_rect[0] + other_rect[2])
                bottom_edge1 = max(rect[3], other_rect[1] + other_rect[3])

                width = right_edge1 - left_edge1  # width and hight in pixels
                height = bottom_edge1 - top_edge1
                if height / width <= max_cone_aspect[colour]:
                    left_edge = left_edge1
                    top_edge = top_edge1
                    right_edge = right_edge1
                    bottom_edge = bottom_edge1
                    # remove smaller box from list
                    rects.remove(other_rect)
                else:
                    i += 1
            else:
                # only increment index if a box wasn't removed from the list
                i += 1

        width = right_edge - left_edge  # width and hight in pixels
        height = bottom_edge - top_edge

        if height / width < min_cone_aspect[colour]:
            height = int(width * min_cone_aspect[colour])
            top_edge = bottom_edge - height

        x_centroid = left_edge + width / 2  # centre of rectangle
        y_centroid = top_edge + height / 2

        # append cone properties to array
        cones.append([col_words[colour], width, height, x_centroid, y_centroid, left_edge, top_edge])

        # draw rectangle with cone geometry
        cv2.rectangle(
            bounding_box_frame,  # image to draw on
            (left_edge, top_edge),
            (right_edge, bottom_edge),  # geometry to draw between
            box_cols[colour],
            2,
        )  # colour and thickness of box (pixels)

        # draw cone centroid dot
        # cv2.circle(bounding_box_frame, (x+w//2, y+h//2), 1, (255, 0, 255), 2)

    return [bounding_box_frame, cones]


def cam_main(frame, display=False):
    # used to crop frame to only required area
    h, w, _ = frame.shape

    # for OG settings script
    if h == 785 and w == 785:
        # mask off sky and car
        roi_mask = np.ones(frame.shape[:2], dtype="uint8")
        cv2.rectangle(roi_mask, (0, 0), (frame.shape[1], 400), 0, -1), [frame.shape[2] - 60, 500]  # mask off sky
        a = 30  # variables for the position of car mask
        b = 100
        c = 235
        h1 = 80
        h2 = 155
        # mask off car
        cv2.fillPoly(
            roi_mask,
            [
                np.array(
                    [
                        [a, frame.shape[1]],
                        [b, frame.shape[1] - h1],
                        [c, frame.shape[1] - h2],
                        [frame.shape[0] / 2, 585],
                        [frame.shape[0] - c, frame.shape[1] - h2],
                        [frame.shape[0] - b, frame.shape[1] - h1],
                        [frame.shape[0] - a, frame.shape[1]],
                    ],
                    np.int32,
                )
            ],
            0,
        )

        current_frame = cv2.bitwise_and(frame, frame, mask=roi_mask)

    # for higher res (variable masking)
    else:
        # vertical res: halfway down bottom
        # horizontal res: full
        current_frame = frame[int(5 * h / 30) : h, 0:w]  # made variable to cam resolution

    bounding_box_frame = np.copy(current_frame)  # frame to display on

    # convert to hsv
    current_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

    cones = list()  # init blank list of cones
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

    try:
        cone_im_size = [100, 100]
        yellow_cones = [cone for cone in cones if cone[0] == "yellow"]
        big_yellow_cone = yellow_cones[0]
        left = int(big_yellow_cone[3] - cone_im_size[0] / 2)
        top = int(big_yellow_cone[4] - cone_im_size[1] / 2)
        right = left + cone_im_size[0]
        bottom = top + cone_im_size[1]

        cropped_img = frame[top:bottom, left:right]
        cv2.imshow("camera-base", cropped_img)  # image with bounding boxes

        im_name = 1
        saved_images = os.listdir("/home/developer/driverless_ws/src/images")
        saved_images.sort()
        if len(saved_images) > 0:
            im_name = int(saved_images[-1][0:5]) + 1
        print(cv2.imwrite("/home/developer/driverless_ws/src/images/{:05}.jpg".format(im_name), cropped_img), im_name)

    except Exception as e:
        print(e)

    try:
        cone_im_size = [100, 100]
        yellow_cones = [cone for cone in cones if cone[0] == "blue"]
        big_yellow_cone = yellow_cones[0]
        left = int(big_yellow_cone[3] - cone_im_size[0] / 2)
        top = int(big_yellow_cone[4] - cone_im_size[1] / 2)
        right = left + cone_im_size[0]
        bottom = top + cone_im_size[1]

        cropped_img = frame[top:bottom, left:right]
        cv2.imshow("camera-base2", cropped_img)  # image with bounding boxes

        im_name = 1
        saved_images = os.listdir("/home/developer/driverless_ws/src/images")
        saved_images.sort()
        if len(saved_images) > 0:
            im_name = int(saved_images[-1][0:5]) + 1
        print(cv2.imwrite("/home/developer/driverless_ws/src/images/{:05}.jpg".format(im_name), cropped_img), im_name)

    except Exception as e:
        print(e)

    ############################################################
    ####### Start of a bunch of keypoint regression shit ####### - current not actualy used for anything
    ############################################################

    # blue_cones = [cone for cone in cones if cone[0] == "orange"]
    # big_blue_cone = blue_cones[0][1:]
    # # big_blue_cone = random.choice(blue_cones)[1:]
    # big_blue_mask = np.zeros(frame.shape[:2], dtype="uint8")
    # # big_blue_mask = cv2.rectangle(big_blue_mask, (big_blue_cone[-2], big_blue_cone[-1]), (big_blue_cone[-2]+big_blue_cone[0], big_blue_cone[-1]+big_blue_cone[1]), 1, -1)
    # cv2.rectangle(big_blue_mask,  (big_blue_cone[-2], big_blue_cone[-1]), (big_blue_cone[-2]+big_blue_cone[0], big_blue_cone[-1]+big_blue_cone[1]), 1, -1)
    # bounding_kernal = np.ones((15, 25), dtype="uint8")
    # big_blue_mask = cv2.dilate(big_blue_mask , bounding_kernal)

    # img = np.copy(current_frame)
    # h_off = 5
    # v_off = 5
    # snip = img[big_blue_cone[-1]-v_off:big_blue_cone[-1]+big_blue_cone[1]+v_off, big_blue_cone[-2]-h_off:big_blue_cone[-2]+big_blue_cone[0]+h_off, :]

    # ########### Harris ###########
    # # img = np.copy(current_frame)#cv2.bitwise_and(frame, frame, mask=big_blue_mask))
    # # gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    # # gray = np.float32(gray)
    # # dst = cv2.cornerHarris(gray,2,3,0.04)
    # # # result is dilated for marking the corners, not important
    # # dst = cv2.dilate(dst,None)
    # # # threshold for an optimal value, it may vary depending on the image.
    # # img[dst>0.02*dst.max()]=[0,0,255]
    # #
    # # # cv2.imshow('dst',dst/dst.max()*255)
    # # # cv2.imshow('dst',cv2.bitwise_and(img, img, mask=big_blue_mask))
    # # cv2.imshow("Harris", img)

    # ########## Shi-Tomasi ###########
    # img = np.copy(current_frame)
    # gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    # corners = cv2.goodFeaturesToTrack(gray,100,0.01,5)
    # corners = np.int0(corners)
    # for i in corners:
    #     x,y = i.ravel()
    #     cv2.circle(img,(x,y),3,(0, 0, 255),-1)
    # # cv2.imshow("Shi-Tomasi", img)

    # ########### SIFT ########### needs opencv-contrib-python (pip install opencv-contrib-python)
    # # img = np.copy(current_frame)
    # # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # # sift = cv2.xfeatures2d.SIFT_create()
    # # detector = sift.detect(gray, None)
    # # kpts, des = sift.compute(gray, detector)
    # # # kpts,des=descriptor.compute(gray,kpts)
    # # im_with_keypoints = cv2.drawKeypoints(gray, kpts, np.array([]), color=255, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # # cv2.imshow("SIFT", im_with_keypoints)

    # ########### SURF ########### needs opencv-contrib-python (pip install opencv-contrib-python)
    # # img = np.copy(current_frame)
    # # surf = cv2.xfeatures2d.SURF_create(50000)
    # # kp, des = surf.detectAndCompute(img,None)
    # # img = cv2.drawKeypoints(img,kp,None,(255,0,0),4)
    # # cv2.imshow("SURF", img)

    # ########### FAST ########### needs opencv-contrib-python (pip install opencv-contrib-python)
    # # img = np.copy(current_frame)
    # # # Initiate FAST object with default values
    # # fast = cv2.FastFeatureDetector()
    # # # find and draw the keypoints
    # # kp = fast.detect(img,None)
    # # img2 = cv2.drawKeypoints(img, kp, color=(255,0,0))
    # # cv2.imshow("FAST", img2)

    # ########### BRIEF ########### needs opencv-contrib-python (pip install opencv-contrib-python)
    # # img = np.copy(current_frame)
    # # # Initiate STAR detector
    # # star = cv2.FeatureDetector_create("STAR")
    # # # Initiate BRIEF extractor
    # # brief = cv2.DescriptorExtractor_create("BRIEF")
    # # # find the keypoints with STAR
    # # kp = star.detect(img,None)
    # # # compute the descriptors with BRIEF
    # # kp, des = brief.compute(img, kp)
    # # img2 = cv2.drawKeypoints(img, kp, color=(255,0,0))
    # # cv2.imshow("FAST", img2)

    # ########### ORB ########### needs opencv-contrib-python (pip install opencv-contrib-python)
    # # img = np.copy(current_frame)
    # # # Initiate STAR detector
    # # orb = cv2.ORB()
    # # # find the keypoints with ORB
    # # kp = orb.detect(img,None)
    # # # compute the descriptors with ORB
    # # kp, des = orb.compute(img, kp)
    # # img2 = cv2.drawKeypoints(img, kp, color=(255,0,0))
    # # cv2.imshow("FAST", img2)

    # # TODO: try Feature Matching here - https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_feature2d/py_matcher/py_matcher.html

    # cv2.waitKey(1)

    if display == True or True:
        # show bounding boxes
        cv2.imshow("camera-base1", bounding_box_frame)  # image with bounding boxes

        cv2.waitKey(1)

    return cones
