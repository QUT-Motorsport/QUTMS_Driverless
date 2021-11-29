import numpy as np

from typing import List

class Threshold:
    lower: np.array
    upper: np.array

    def __init__(self, lower: List[int], upper: List[int]) -> None:
        self.lower = np.array(lower, np.uint8)
        self.upper = np.array(upper, np.uint8)
