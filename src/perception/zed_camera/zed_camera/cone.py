from dataclasses import dataclass

from .rect import Rect

@dataclass
class Cone:
    bounding_box: Rect
    distance: float
