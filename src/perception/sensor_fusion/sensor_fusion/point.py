from dataclasses import dataclass
import numpy as np
from typing import Tuple
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
from math import sqrt, sin, cos

@dataclass
class Point:
    x: float
    y: float

    def __add__(self, other: "Point") -> "Point":
        return Point(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Point") -> "Point":
        return Point(self.x - other.x, self.y - other.y)

    def __truediv__(self, divisor: int) -> "Point":
        return Point(int(round(self.x/divisor)), int(round(self.y/divisor)))
    
    # NEW METHODS ADDED
    def __mul__(self, multiplier: int) -> "Point":
        return Point(self.x*multiplier, self.y*multiplier)
    
    def to_tuple(self) -> Tuple:
        return (self.x, self.y)


class PointWithCov:
    def __init__(self,
        loc_x: float,
        loc_y: float,
        loc_z: float,
        loc_cov: np.array,
        color: int,
        header: Header,
        global_x: float = None,
        global_y: float = None,
        global_z: float = None,
        global_cov: np.array = None
        ) -> None:
        
        self.loc_x: float = loc_x
        self.loc_y: float = loc_y
        self.loc_z: float = loc_z
        self.loc_cov: np.array = loc_cov
        self.color: int = color
        self.isyellow = 0
        self.isblue = 0
        self.isorange = 0
        self.isbig = 0
        self.issmall = 0
        self.global_x: float = global_x
        self.global_y: float = global_y
        self.global_z: float = global_z
        self.global_cov: np.array = global_cov
        self.coords = (self.global_x, self.global_y)
        self.header: Header = header
        self.nMeasurments: int = 0
        # have to start with 1 so we dont get a div by zero error
        self.ncMeasurments: int = 1
    
    def updatecolor(self, color):
        if color == 0:
            self.isblue += 1
            self.ncMeasurments += 1
        elif color == 1:
            self.isyellow += 1
            self.ncMeasurments += 1
        elif color == 2:
            self.isorange += 1
            self.isbig += 1
            self.ncMeasurments += 1
        elif color == 3:
            self.isorange += 1
            self.issmall += 1
            self.ncMeasurments += 1
        else:
            pass
        if self.isblue / self.ncMeasurments > 0.5:
            self.color = 0
        elif self.isyellow / self.ncMeasurments > 0.5:
            self.color = 1
        elif self.isorange / self.ncMeasurments > 0.5:
            if self.isbig / (self.isbig+self.issmall) > 0.5:
                self.color = 2
            else:
                self.color = 3
        else:
            self.color = 4

    def translate(self, x, y, z, theta, g_cov):
        s, c = sin(theta), cos(theta)
        rotation_matrix = np.array([[c, -1*s, 0],[s, c, 0], [0, 0, 1]])
        new_cov = rotation_matrix @ self.loc_cov @ rotation_matrix.T
        self.global_cov = new_cov + g_cov
        self.global_x = x + self.loc_x * c - self.loc_y * s
        self.global_y = y + self.loc_y * c + self.loc_x * s
        self.global_z = z + self.loc_z
        self.coords = (self.global_x, self.global_y)

    def update(self, other:"PointWithCov"):
        m3, c3 = multivariate_multiply([self.global_x, self.global_y, self.global_z], self.global_cov, [other.global_x, other.global_y, other.global_z], other.global_cov)
        self.global_cov = c3
        self.global_x = m3[0]
        self.global_y = m3[1]
        self.global_z = m3[2]
        self.coords = (self.global_x, self.global_y)
        self.nMeasurments += 1
        self.updatecolor(other.color)

    def covMax(self, lim):
        return sqrt(self.global_cov[0,0]**2+self.global_cov[1,1]**2+self.global_cov[2,2]**2) < lim

    def inTwoSigma(self, other:"PointWithCov"):
        # get the vector between the points
        vector = [self.global_x-other.global_x, self.global_y-other.global_y, self.global_z-other.global_z]
        # get the normalized vector between the points
        dist = sqrt(vector[0]**2+vector[1]**2+vector[2]**2)+ 0.00001
        # calculate the distance fo 2 sigma in the direction of the normal vector
        selfTwosig = sqrt(((vector[0] / dist) * sqrt(abs(self.global_cov[0,0])))**2 + ((vector[1] / dist) * sqrt(abs(self.global_cov[1,1])))**2 + ((vector[2] / dist) * sqrt(abs(self.global_cov[2,2])))**2) * 2
        otherTwosig = sqrt(((vector[0] / dist) * sqrt(abs(other.global_cov[0,0])))**2 + ((vector[1] / dist) * sqrt(abs(other.global_cov[1,1])))**2 + ((vector[2] / dist) * sqrt(abs(other.global_cov[2,2])))**2) * 2
        # see if the distance is less than the sum of the two 2 sigma vectors
        return dist < selfTwosig + otherTwosig

    def inFourSigma(self, other:"PointWithCov"):
        # get the vector between the points
        vector = [self.global_x-other.global_x, self.global_y-other.global_y, self.global_z-other.global_z]
        # get the normalized vector between the points
        dist = sqrt(vector[0]**2+vector[1]**2+vector[2]**2)+ 0.00001
        # calculate the distance fo 4 sigma in the direction of the normal vector
        selfTwosig = sqrt(((vector[0] / dist) * sqrt(abs(self.global_cov[0,0])))**2 + ((vector[1] / dist) * sqrt(abs(self.global_cov[1,1])))**2 + ((vector[2] / dist) * sqrt(abs(self.global_cov[2,2])))**2) * 4
        otherTwosig = sqrt(((vector[0] / dist) * sqrt(abs(other.global_cov[0,0])))**2 + ((vector[1] / dist) * sqrt(abs(other.global_cov[1,1])))**2 + ((vector[2] / dist) * sqrt(abs(other.global_cov[2,2])))**2) * 4
        # see if the distance is less than the sum of the two 2 sigma vectors
        return dist < selfTwosig + otherTwosig


    def dist(self, other:"PointWithCov"):
        return sqrt((self.global_x-other.global_x)**2+(self.global_y-other.global_y)**2+(self.global_z-other.global_z)**2)

    # should add cone color to this
    def getMarker(self, id: int):
        return point_msg(self.global_x, self.global_y, self.global_z, id, self.color)

    def getCov(self, id: int, buffer: bool):
        # make a deformed sphere at 3 sigma of the variance in each axis (the diagnal elements of the covariance matrix are squared so we gotta sqrt)
        return cov_msg(self.global_x, self.global_y, self.global_z, id, 3*sqrt(abs(self.global_cov[0,0])), 3*sqrt(abs(self.global_cov[1,1])), 3*sqrt(abs(self.global_cov[2,2])), buffer)

    def __len__(self):
        return len(self.coords)

    def __getitem__(self, i):
        return self.coords[i]

    def __repr__(self):
        return 'Item({}, {}, {})'.format(self.coords[0], self.coords[1], self.color)

    def __eq__(self, other:"PointWithCov"):
        return self.global_x == other.global_x and self.global_y == other.global_y


def point_msg(
    x_coord: float, 
    y_coord: float, 
    z_coord: float,
    ID: int, 
    color: int
) -> Marker: 

    marker = Marker()
    marker.header.frame_id = "map"
    marker.ns = "point_markers"
    marker.id = ID
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.pose.position.x = x_coord
    marker.pose.position.y = y_coord
    marker.pose.position.z = z_coord
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # scale out of 1x1x1m
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    marker.color.a = 1.0 # alpha
    # make the cone its own color and black if unknown
    if color == 0:
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
    elif color == 1:
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
    elif color == 2 or color == 3:
        marker.color.r = 1.0
        marker.color.g = 0.7
        marker.color.b = 0.0
    else:
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0

    marker.lifetime = Duration(sec=0, nanosec=100000000)

    return marker


def cov_msg(
    x_coord: float, 
    y_coord: float, 
    z_coord: float,
    ID: int, 
    x_scale: float,
    y_scale: float,
    z_scale: float,
    buffer: bool
) -> Marker: 

    marker = Marker()
    marker.header.frame_id = "map"
    marker.ns = "covariance_markers"
    marker.id = ID
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.pose.position.x = x_coord
    marker.pose.position.y = y_coord
    marker.pose.position.z = z_coord
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # scale out of 1x1x1m
    marker.scale.x = x_scale
    marker.scale.y = y_scale
    marker.scale.z = z_scale

    marker.color.a = 0.3 # alpha
    if buffer:
        marker.color.r = 0.65
        marker.color.g = 0.65
        marker.color.b = 0.0
    else:
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

    marker.lifetime = Duration(sec=0, nanosec=500000000)

    return marker


def multivariate_multiply(m1, c1, m2, c2):
    """
    Taken from the filterpy library
    MIT Licence
    https://github.com/rlabbe/filterpy
    Multiplies the two multivariate Gaussians together and returns the
    results as the tuple (mean, covariance).
    Examples
    --------
    .. code-block:: Python
        m, c = multivariate_multiply([7.0, 2], [[1.0, 2.0], [2.0, 1.0]],
                                     [3.2, 0], [[8.0, 1.1], [1.1,8.0]])
    Parameters
    ----------
    m1 : array-like
        Mean of first Gaussian. Must be convertable to an 1D array via
        numpy.asarray(), For example 6, [6], [6, 5], np.array([3, 4, 5, 6])
        are all valid.
    c1 : matrix-like
        Covariance of first Gaussian. Must be convertable to an 2D array via
        numpy.asarray().
     m2 : array-like
        Mean of second Gaussian. Must be convertable to an 1D array via
        numpy.asarray(), For example 6, [6], [6, 5], np.array([3, 4, 5, 6])
        are all valid.
    c2 : matrix-like
        Covariance of second Gaussian. Must be convertable to an 2D array via
        numpy.asarray().
    Returns
    -------
    m : ndarray
        mean of the result
    c : ndarray
        covariance of the result
    """

    C1 = np.asarray(c1)
    C2 = np.asarray(c2)
    M1 = np.asarray(m1)
    M2 = np.asarray(m2)

    sum_inv = np.linalg.inv(C1+C2)
    C3 = np.dot(C1, sum_inv).dot(C2)

    M3 = (np.dot(C2, sum_inv).dot(M1) +
          np.dot(C1, sum_inv).dot(M2))

    return M3, C3