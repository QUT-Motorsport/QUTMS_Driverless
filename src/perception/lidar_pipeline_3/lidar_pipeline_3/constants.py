from enum import Enum
import math
import pathlib

# Config Parameters
WORKING_DIR: str = str(pathlib.Path(__file__).parent.resolve())
OUTPUT_DIR: str = WORKING_DIR + "/output"
FIGURES_DIR: str = "/figures"

# Algorithm Parameters
LIDAR_RANGE = 22.5  # Max range of points to process (metres)
DELTA_ALPHA = (2 * math.pi) / 128  # Delta angle of segments
BIN_SIZE = 0.14  # Size of bins

# Visualiser
# Default Values
dark_grey = (0.122, 0.149, 0.188, 1.0)  # 1f2630
light_grey = (0.145, 0.181, 0.247, 1.0)  # 252e3f
blue = (0.612, 0.863, 0.996, 1.0)  # 9cdcfe
mint = (0.188, 0.992, 0.761, 1.0)  # 30fdc3

# Custom Colours
mint_hex = "#30fdc3"
yellow_hex = "#F4D44D"
red_hex = "#F45060"
blue_hex = "#636EFA"

# Colour Sets
colours_01 = [mint_hex, yellow_hex, red_hex, blue_hex]
colours_01 = [blue_hex, yellow_hex, red_hex]
