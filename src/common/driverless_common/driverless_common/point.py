from dataclasses import dataclass
from math import sqrt, pi

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header

from driverless_msgs.msg import Cone

from driverless_common.common import angle, orientation, wrap_to_pi


@dataclass
class Point:
    x: float
    y: float

    def __add__(self, other: "Point") -> "Point":
        return Point(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Point") -> "Point":
        return Point(self.x - other.x, self.y - other.y)

    def __truediv__(self, divisor: int) -> "Point":
        return Point(int(round(self.x / divisor)), int(round(self.y / divisor)))

    # NEW METHODS ADDED
    # def __mul__(self, multiplier: int) -> "Point":
    #     return Point(self.x * multiplier, self.y * multiplier)

    def __mul__(self, multiplier: float) -> "Point":
        return Point(self.x * multiplier, self.y * multiplier)

    def __repr__(self) -> str:
        print(f"({self.x}, {self.y})")
    
    def __getitem__(self, index: int) -> float:
        return self.to_tuple()[index]

    def to_tuple(self) -> tuple:
        return (self.x, self.y)

def dist(a: Point, b: Point) -> float:
    return sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


def cone_to_point(cone: Cone) -> Point:
    return Point(
        cone.location.x,
        cone.location.y,
    )

def points_to_path(points: list[Point], header: Header) -> Path:
    spline_len = len(points)
    poses: list[PoseStamped] = []  # target spline poses
    for i in range(spline_len):
        # get angle between current point and next point
        if i < spline_len - 1:
            th_change = angle(points[i], points[i + 1])
        elif i == spline_len - 1:
            th_change = angle(points[i], points[0])
        th_change = wrap_to_pi(th_change) # keep between 360

        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = float(points[i].x)
        pose.pose.position.y = float(points[i].y)
        pose.pose.position.z = 0.0
        pose.pose.orientation = orientation(0, 0, th_change)
        poses.append(pose)

    # Add the first path point to the end of the list to complete the loop
    # poses.append(poses[0])
    path_msg = Path()
    path_msg.header = header
    path_msg.poses = poses
    return path_msg
