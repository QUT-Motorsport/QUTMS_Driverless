# import python libraries 
from math import sqrt
from typing import List
 
# finds distances (magnitude) between two top-down points 
def distance(x1: float, y1: float, x2: float, y2: float):  
    return sqrt((x1-x2)**2 + (y1-y2)**2)  
 
# averages all x,y points that hit 1 cone to find the cone's centre coord 
def pointgroup_to_cone(group: List[float]): 
    average_x: float = 0 
    average_y: float = 0 
    average_z: float = 0 
 
    for point in group: 
        average_x += point[0] 
        average_y += point[1] 
        average_z += point[2] 
        # average_i += point[3] # intensity value 
    average_x = average_x / len(group) 
    average_y = average_y / len(group) 
    average_z = average_z / len(group) 
    average_i = 0 # will change 
    # average_i = average_i / len(group) 
    return [float(average_x), float(average_y), float(average_z), float(average_i)] 
 
# evaluate all points and find points that are grouped together as those will probably be cones. 
def find_cones(points: List[list], max_range: float = 18.0, distance_cutoff: float = 0.1): 
    current_group: list = [] 
    cones: list = [] 
 
    for i in range(1, len(points)):
        # Get the distance from current to previous point 
        distance_to_last_point: float = distance(points[i][0], points[i][1], points[i-1][0], points[i-1][1]) 
 
        if distance_to_last_point < distance_cutoff: 
            # Points closer together then the cutoff are part of the same group = same cone 
            current_group.append([points[i][0], points[i][1], points[i][2]]) 
 
        else: 
            # points further away indiate a split between groups 
            if len(current_group) > 0: 
                cone = pointgroup_to_cone(current_group) 

                distance_to_cone: float = distance(cone[0], cone[1], 0, 0) 
                if distance_to_cone < max_range:
                    cones.append(cone) 
 
                current_group: list = [] 
 
    return cones 