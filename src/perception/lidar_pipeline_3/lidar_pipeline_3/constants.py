from enum import Enum
import pathlib

# Config Parameters
WORKING_DIR: str = str(pathlib.Path(__file__).parent.resolve())
OUTPUT_DIR: str = WORKING_DIR + "/output"
FIGURES_DIR: str = "/figures"

# Algorithm Parameters
LIDAR_RANGE = 22.5  # Max range of points to process (metres)

# Misc
class Colour(Enum):
    MINT = (0.188, 0.992, 0.761, 1.0)  # 30fdc3
    BLUE = (0.612, 0.863, 0.996, 1.0)  # 9cdcfe
    LIGHT_GREY = (0.145, 0.181, 0.247, 1.0)  # 252e3f
    DARK_GREY = (0.122, 0.149, 0.188, 1.0)  # 1f2630
