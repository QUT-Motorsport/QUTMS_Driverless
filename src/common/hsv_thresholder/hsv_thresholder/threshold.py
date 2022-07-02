# COPIED FROM ZED_CAMERA
# TODO: figure out a way to share python code among ROS packages

import json
import numpy as np

from typing import List


class Threshold:
    lower: np.array
    upper: np.array

    def __init__(self, lower: List[int], upper: List[int]) -> None:
        self.lower = np.array(lower, np.uint8)
        self.upper = np.array(upper, np.uint8)

    # NEW METHODS ADDED
    def to_json(self) -> str:
        json_obj = {
            "lower": self.lower.tolist(),
            "upper": self.upper.tolist(),
        }
        return json.dumps(json_obj)

    @classmethod
    def from_json(cls, json_str: str) -> "Threshold":
        json_obj = json.loads(json_str)
        return cls(
            lower=json_obj["lower"],
            upper=json_obj["upper"],
        )
