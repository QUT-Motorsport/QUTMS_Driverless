# import python libraries
import math

## helper function to find distances (magnitude) between two top-down points
def distance(x1, y1, x2, y2): 
    distance = math.sqrt(math.pow(abs(x1-x2), 2) + math.pow(abs(y1-y2), 2))
    return distance

## helper function to find the average y "centre" of the cones. this is calculated wrt the FOV centre
def find_avg_y(cone_set):
    length = len(cone_set)
    if length != 0:
        average_y = 0
        for cone in cone_set:
            average_y += cone[1]
        average_y = average_y / length

        return -average_y
    
    else:
        return 0 
