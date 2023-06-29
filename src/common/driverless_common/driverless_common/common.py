from math import atan2, sqrt

import numpy as np

from typing import List, Tuple


def dist(p1: List[float], p2: List[float]) -> float:
    """
    Retrieve distance between two points
    * param p1: [x,y] coords of point 1
    * param p2: [x,y] coords of point 2
    * return: distance between points
    """
    return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def fast_dist(p1: List[float], p2: List[float]) -> float:
    """
    Calculates the distance between two 2-dimensional points using the euclidian distance formula, but doesn't sqrt the
    result, therefore the result of this function is the distance between the two points squared. This should only be
    used for operations that compare distances created by this function, for example, if you  have a point, and need to
    find the closest point to it from a list, you should use this function to calculate the distances for each point in
    the list, they can be compared to each other as they are all squared. Sqrts are very expensive calculations, and
    this function allows us to avoid performing them n times for an n sized list.

    Be careful what you compare the reuslt of this function against, if you are checking that for points in a list that
    are over or under a certain distance, you must square the distance before the comparison.
    """
    diff_x = p1[0] - p2[0]
    diff_y = p1[1] - p2[1]
    return diff_x * diff_x + diff_y * diff_y


def angle(p1: List[float], p2: List[float]) -> float:
    """
    Retrieve angle between two points
    * param p1: [x,y] coords of point 1
    * param p2: [x,y] coords of point 2
    * return: angle in rads
    """
    x_disp = p2[0] - p1[0]
    y_disp = p2[1] - p1[1]
    return atan2(y_disp, x_disp)


def wrap_to_pi(angle: float) -> float:
    """
    Wrap an angle between -pi and pi
    * param angle: angle in rads
    * return: angle in rads wrapped to -pi and pi
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi


def midpoint(p1: List[float], p2: List[float]) -> Tuple[float]:
    """
    Retrieve midpoint between two points
    * param p1: [x,y] coords of point 1
    * param p2: [x,y] coords of point 2
    * return: x,y tuple of midpoint coord
    """
    return (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2
