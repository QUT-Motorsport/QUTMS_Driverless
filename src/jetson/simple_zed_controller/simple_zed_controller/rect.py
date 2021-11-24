import cv2
from dataclasses import dataclass
import numpy as np

from typing import Tuple

@dataclass
class Point:
    x: int
    y: int

    def __add__(self, other: "Point") -> "Point":
        return Point(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Point") -> "Point":
        return Point(self.x - other.x, self.y - other.y)

    def __truediv__(self, divisor: int) -> "Point":
        return Point(int(round(self.x/divisor)), int(round(self.y/divisor)))

@dataclass
class Rect:
    tl: Point
    width: int
    height: int

    def __init__(self, x: int, y: int, width: int, height: int) -> None:
        self.tl = Point(x, y)
        self.width = width
        self.height = height

    @property
    def bl(self) -> Point:
        return Point(self.tl.x, self.tl.y + self.height)

    @property
    def br(self) -> Point:
        return Point(self.tl.x + self.width, self.tl.y + self.height)
    
    @property
    def bc(self) -> Point:
        return Point(self.tl.x + int(round(self.width / 2)), self.tl.y + self.height)

    @property
    def center(self) -> Point:
        return Point(int(round(self.tl.x + self.width / 2)), int(round(self.tl.y + self.height / 2)))

    @property
    def area(self) -> int:
        return self.width * self.height


def draw_box(img: np.ndarray, box: Rect, colour: Tuple):
    # draw a bounding box around the image and display it
    cv2.rectangle(img, (box.tl.x, box.tl.y), (box.tl.x + box.width, box.tl.y + box.height), colour, thickness=2)
