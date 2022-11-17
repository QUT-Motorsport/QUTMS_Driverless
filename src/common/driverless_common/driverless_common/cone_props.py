from dataclasses import dataclass
import math

from driverless_msgs.msg import Cone

from typing import Tuple


@dataclass
class ConeProps:
    """Creates a datatype for storing and converting cones recorded"""

    x: float
    y: float
    colour: int
    range: float
    bearing: float

    def __init__(self, cone: Cone, sensor: str):
        if sensor == "lidar":
            self.x = cone.location.x + 1.65
        else:
            self.x = cone.location.x - 0.1
        self.y = cone.location.y
        self.colour = cone.color

        self.range = math.hypot(self.x, self.y)
        self.bearing = math.atan2(self.y, self.x)

    @property
    def sense_rb(self) -> Tuple[float, float]:
        return self.range, self.bearing
