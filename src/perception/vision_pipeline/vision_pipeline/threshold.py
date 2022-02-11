import numpy as np

from typing import List

class Threshold:
    """
    Creates a Numpy array object for HSV mask value pairs
    """
    lower: np.array
    upper: np.array

    def __init__(self, lower: List[int], upper: List[int]) -> None:
        self.lower = np.array(lower, np.uint8)
        self.upper = np.array(upper, np.uint8)
