from dataclasses import dataclass
import math

from .GPE2 import BIN_SIZE, NUM_BINS, DELTA_ALPHA

@dataclass
class Point_Obj:
    x: float
    y: float
    z: float
    i: float
    bin: int
    segment: int
    is_ground: bool

    def __init__(self, x: float, y: float, z: float, intensity: float):
        self.x = x
        self.y = y
        self.z = z
        self.i = intensity
        self.bin = None
        self.segment = None
        self.is_ground = None

    @property
    def xy_distance(self) -> float:
        return math.sqrt( (self.x**2) + (self.y**2) )

    @property
    def xyz_distance(self) -> float:
        return math.sqrt( (self.x**2) + (self.y**2) + (self.z**2) )
    
    # Returns the index to a bin that a point (x, y) maps to 
    @property
    def bin(self) -> int:
        norm = math.sqrt( (self.x**2) + (self.y**2) )
        bin_index = math.floor(norm / BIN_SIZE)
        if norm % BIN_SIZE != 0:
            bin_index += 1
        if bin_index >= NUM_BINS:
            bin_index = -1
            #print("Point exceeds expected max range of LIDAR. bin_index:", bin_index)
        return math.floor(bin_index)

    # Returns the index to a segment that a point maps to
    @property
    def segment(self) -> int:
        return math.floor(math.atan2(self.y, self.x) / DELTA_ALPHA)