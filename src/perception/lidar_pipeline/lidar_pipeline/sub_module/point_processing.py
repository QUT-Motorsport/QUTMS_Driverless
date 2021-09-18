""" Add the following settings to Lidar1 or Lidar2

"<lidar name>": {
    "SensorType": 6,
    "Enabled": true,
    "X": 1.3, "Y": 0, "Z": 0.1,
    "Roll": 0, "Pitch": 0, "Yaw" : 0,
    "NumberOfLasers": 1,
    "PointsPerScan": 500,
    "VerticalFOVUpper": 0,
    "VerticalFOVLower": 0,
    "HorizontalFOVStart": -90,
    "HorizontalFOVEnd": 90,
    "RotationsPerSecond": 10,
    "DrawDebugPoints": true
}
"""

# import python libraries
import math

# finds distances (magnitude) between two top-down points
def distance(x1, y1, x2, y2): 
    distance = math.sqrt(math.pow(abs(x1-x2), 2) + math.pow(abs(y1-y2), 2))
    return distance

# averages all x,y,z and intensity points that hit 1 cone to find centroid
def pointgroup_to_cone(group):
    average_x = 0 # init avgs
    average_y = 0
    average_z = 0

    g_len = len(group) # length of point group
    # iterate through each point in group
    for i in range(0, g_len):
        point = group[i]
        average_x += point[0] # increase sum
        average_y += point[1]
        average_z += point[2]
        # average_i += point[3] # intensity value
    average_x = average_x / g_len # calculate avgs
    average_y = average_y / g_len
    average_z = average_z / g_len
    # average_i = average_i / g_len

    c = 0 # no colour data from sim atm
    # colours would be 0 = blue, 1 = yellow, 2 = orange 

    return [float(average_x), float(average_y), float(average_z), int(c)]

# evaluate all points and find points that are grouped together as those will probably be cones.
def find_cones(points, max_range_cutoff, distance_cutoff):
    current_group = list() # init groups and cones
    cones = list()

    # iterate through each point
    for i in range(1, len(points)):
        # distance_to_point = distance(0, 0, points[i][0], points[i][1])

        # if distance_to_point < max_range_cutoff:
        
        # Get the distance from current to previous point
        distance_to_last_point = distance(points[i][0], points[i][1], points[i-1][0], points[i-1][1])

        if distance_to_last_point < distance_cutoff:
            # Points closer together then the cutoff are part of the same group = same cone
            current_group.append([points[i][0], points[i][1], points[i][2]])

        else:
            # points further away indiate a split between groups
            if len(current_group) > 0:
                cone = pointgroup_to_cone(current_group)
                cones.append(cone)

                current_group = list() # clear group

    return cones
