import cv2
import numpy as np

from typing import List

from .rect import Rect
from .threshold import Threshold

Contour = List

kernal = np.ones((5, 5), "uint8")

def get_coloured_objects(hsv_img: np.ndarray, thresholds: List[Threshold]) -> List[Rect]:
    contours = get_coloured_contours(hsv_img, thresholds)
    return get_large_contour_bounds(contours)

def get_coloured_contours(hsv_img: np.ndarray, thresholds: List[Threshold]) -> List[Contour]:
    masks = [cv2.dilate(cv2.inRange(hsv_img, threshold.lower, threshold.upper), kernal) for threshold in thresholds]

    full_mask = sum(masks)

    contours, _ = cv2.findContours(full_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def get_large_contour_bounds(contours: List[Contour], thresh: int = 100) -> List[Rect]:
    return [Rect(*cv2.boundingRect(c)) for c in contours if cv2.contourArea(c) > thresh]