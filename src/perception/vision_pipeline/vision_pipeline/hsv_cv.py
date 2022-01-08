# python libraries
import cv2
import numpy as np
from typing import List

# import required sub modules
from .rect import Rect
from .threshold import Threshold

Contour = List

kernal = np.ones((5, 5), "uint8") # used for dilating mask

def get_coloured_contours(hsv_img: np.ndarray, threshold: Threshold) -> List[Contour]:
    """
    Returns list of contours for 1 colour by:
    - Masking the frame for the current colour's threshold bounds
    - Extracting the contours of the mask
    """
    mask = cv2.dilate(cv2.inRange(hsv_img, threshold.lower, threshold.upper), kernal)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours


def get_large_contour_bounds(contours: List[Contour]) -> List[Rect]:
    """
    Returns array of Rect objects by:
    - Computing each bounding rectangle around each contour object
    """
    return [Rect(*cv2.boundingRect(c)) for c in contours]


def get_coloured_bounding_boxes(hsv_img: np.ndarray, threshold: Threshold) -> List[Rect]:
    """
    Returns list of bounding boxes for a colour by:
    - Getting contours
    - Getting bounding rectangles
    """
    contours = get_coloured_contours(hsv_img, threshold)
    return get_large_contour_bounds(contours)
