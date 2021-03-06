from dataclasses import dataclass
import math
from turtle import color

from driverless_msgs.msg import Cone

from typing import Optional, Tuple


@dataclass
class MapCone:
    """Creates a datatype for storing and converting cones recorded"""

    x: float
    y: float
    colour: Tuple[int, int, int]
    range: float
    bearing: float

    def __init__(self, cone: Cone):
        self.x = cone.location.x
        self.y = cone.location.y
        self.colour = cone.color

        self.range = math.hypot(self.x, self.y)
        self.bearing = math.atan2(self.y, self.x)

    @property
    def sense_rb(self) -> Tuple[float, float]:
        return self.range, self.bearing
