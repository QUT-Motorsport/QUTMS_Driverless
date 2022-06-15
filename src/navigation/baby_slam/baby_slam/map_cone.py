from dataclasses import dataclass
from turtle import color
from typing import Tuple, Optional
import math

from driverless_msgs.msg import Cone

@dataclass
class MapCone:
    """Creates a datatype for storing and converting cones recorded"""

    def __init__(self, cone: Cone):
        self.x = cone.location.x
        self.y = cone.location.y
        self.colour = cone.color

        self.range = math.hypot(self.x, self.y)
        self.bearing = math.atan2(self.y, self.x)

    @property
    def sense_rb(self) -> Tuple:
        return (self.range, self.bearing)