# python libraries
import cv2
from dataclasses import dataclass
import numpy as np
from typing import Tuple, Optional

@dataclass
class Point:
    """
    Creates a datatype for storing a point x,y coordinate
    """
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
    """
    Creates a datatype for storing a rectangle (in terms of pixels)
    - Initialised with 'top-left' pixel coord, width and length in pixels
    
    Available properties:
    - bl:   Return the 'bottom-left' pixel coord
    - br:   Return the 'bottom-right' pixel coord
    - bc:   Return the 'bottom-centre' pixel coord
    - centre:   Return the 'centre' pixel coord
    - area:   Return the area in pixels
    - aspect_ratio:   Return width:height ratio (from 0-1)
    - as_roi:   Specify a rectangle as a region of interest on a frame.
                Returns the region as a sub-frame Numpy array
    """
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
    
    @property
    def aspect_ratio(self) -> float:
        return self.width / self.height

    def as_roi(self, frame: np.ndarray) -> np.ndarray:
        return frame[self.tl.y:self.br.y, self.tl.x:self.br.x]


def draw_box(img: np.ndarray, box: Rect, colour: Tuple, distance: Optional[float] = None):
    """
    Draws a box on a display image, around a given rectangle object, of the cone's colour\n
    Also writes the distance to the cone in meters if it is a valid distance
    """

    # draw a bounding box around the cone on the display image
    cv2.rectangle(
        img, 
        (box.tl.x, box.tl.y), 
        (box.tl.x + box.width, box.tl.y + box.height), 
        colour, 
        thickness=2
    )

    # write text above the cone bounding box on the display image
    if distance is not None:
        cv2.putText(
            img,
            str(round(distance, 1)) + "m",
            (box.tl.x, box.tl.y - 3),
            cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.4,
            color=(255, 255, 255),
            thickness=1
        )
