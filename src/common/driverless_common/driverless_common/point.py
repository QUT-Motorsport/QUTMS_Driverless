from dataclasses import dataclass
from math import sqrt

from driverless_msgs.msg import Cone

from typing import Tuple


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
    def __mul__(self, multiplier: int) -> "Point":
        return Point(self.x * multiplier, self.y * multiplier)

    def to_tuple(self) -> Tuple:
        return (self.x, self.y)


def dist(a: Point, b: Point) -> float:
    return sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


def cone_to_point(cone: Cone) -> Point:
    return Point(
        cone.location.x,
        cone.location.y,
    )
