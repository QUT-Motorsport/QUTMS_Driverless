import cv2
from dataclasses import dataclass
import numpy as np

from typing import Tuple

@dataclass
class Rect:
    x: int
    y: int
    width: int
    height: int

    @property
    def tl(self) -> Tuple[int, int]:
        return (self.x, self.y)

    @property
    def br(self) -> Tuple[int, int]:
        return (self.x + self.width, self.y + self.height)

    @property
    def center(self) -> Tuple[int, int]:
        return (int(round(self.x + self.width / 2)), int(round(self.y + self.height / 2)))

    @property
    def area(self) -> int:
        return self.width * self.height
    
    @property
    def bottom(self) -> int:
        return self.y + self.height


def draw_box(img: np.ndarray, box: Rect, colour: Tuple):
    # draw a bounding box around the image and display it
    cv2.rectangle(img, (box.x, box.y), (box.x + box.width, box.y + box.height), colour, thickness=2)
