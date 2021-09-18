# import python libraries
import cv2

## a function which gets the centre of a bounding box (rectangle in this instance)
def get_centroid(x1, y1, x2, y2):
    xC = (x1 + x2) / 2 
    yC = (y1 + y2) / 2

    xCyC = []
    xCyC.append(xC)
    xCyC.append(yC)

    return xCyC


def get_close(num_cones, results, colour):
    # an array that stores all of the blue traffic cones detected: 
    cones = []
    for i in range(0, num_cones):
        if(results.xyxy[0][i][5] == colour):
            cones.append(results.xyxy[0][i])
    # print('the number of cones is: ',len(cones))

    # calculating the centroids of the cones:
    cone_centroids = []
    for i in range(0, len(cones)):
        cone_centroids.append(get_centroid(cones[i][0], cones[i][1], cones[i][2], cones[i][3]))
    # print('cone centroids', cone_centroids)

    # getting the closet cone (based on y position): 
    closest = []
    if(len(cone_centroids) > 0):
        min_cone = cone_centroids[0][1]
        closest = cone_centroids[0] # set initial closest to the first in array

        for i in range(0, len(cone_centroids) ):
            if(cone_centroids[i][1] > min_cone):
                closest = cone_centroids[i] # define now closest cone
    # print('The closet cone is located at: ', closet)

    return closest


## a function for the main yolov5 detector
def yolov5_detector(model, image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    # inference 
    results = model(image)

    # rendering and getting image list
    im_list = results.render()
    im_list = [x[..., ::-1] for x in im_list]  # RGB to BGR


    # getting the bounding boxes of all the detected cones: 
    # bounding_boxes = results.xyxy[0]
    # print('\n', bounding_boxes)

    # number of cones detected: 
    num_cones = len(results.xyxy[0])
    # print('the number of cones is: ', num_cones)

    blue_res = 0
    yellow_res = 1

    closest_blue = get_close(num_cones, results, blue_res)
    closest_yellow = get_close(num_cones, results, yellow_res)

    # calculating the midpoint of the closet blue and yellow cones:
    if closest_blue != [] and closest_yellow != []:
        targetXY = []
        targetXY = get_centroid(closest_blue[0], closest_blue[1], closest_yellow[0], closest_yellow[1])
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