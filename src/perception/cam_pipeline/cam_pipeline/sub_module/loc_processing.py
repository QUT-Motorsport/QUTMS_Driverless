# import python libraries
import math
import cv2

FOCAL_LEN = 650
SMALL_CONE_W = 0.228
SMALL_CONE_H = 0.505
LARGE_CONE_W = 0.228
LARGE_CONE_H = 0.325


def find_pos(centre, dimension, cone):
# retrieve object properties
    width = cone[1]
    height = cone[2]
    x_centroid = cone[3]
    y_centroid = cone[4]

    # would prefer to use height as it is less sensitive to cone bases on an angle
    object_len = width
   
    # one time operation to find focal length at a known distance
    # focal = (object_len * KNOWN_DIST) / SAMPLE_SIZE
    # print("focal: ", focal)

    # find direct line of sight distance (from top-mounted angled camera)
    x_dist = (dimension[0] * FOCAL_LEN) / object_len
    # find distance along ground (using pythag) when camera is angled down
    # x_dist = math.sqrt(pow(z_dist, 2) - pow(CAM_HEIGHT, 2))

    # find ratio between actual object size and found pixels
    pixel_ratio = dimension[0] / object_len
    # calculate distance (m) to centre using pixel ratio
    error = x_centroid - centre
    y_dist = pixel_ratio * error

    z_dist = pixel_ratio * dimension[1]
    
    return [x_dist, y_dist, z_dist]


# coord system: x: out from camera / robot
#               y: across along camera horizontal axis
#               z: up along camera vertical axis
def display_locations(centre, cones):
    cone_coords = list() # init coord list

    # iterate through each cone
    for i in range(0, len(cones)): 
        cone = cones[i]
        if cone[0] == "blue":
            dimensions = [SMALL_CONE_W, SMALL_CONE_W] # blue cone dimensions
            # call find_pos to calculate xyz coord
            [x, y, z] = find_pos(centre, dimensions, cone)
            colour = 0

        elif cone[0] == "yellow":
            dimensions = [SMALL_CONE_W, SMALL_CONE_W]
            [x, y, z] = find_pos(centre, dimensions, cone)
            colour = 1

        elif cone[0] == "orange":
            dimensions = [LARGE_CONE_W, LARGE_CONE_H]
            [x, y, z] = find_pos(centre, dimensions, cone)
            colour = 2

        # append cone properties to coords list
        cone_coords.append([x, y, z, colour])

    return cone_coords