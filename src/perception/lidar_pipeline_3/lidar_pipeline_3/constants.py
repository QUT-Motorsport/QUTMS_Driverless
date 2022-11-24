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
T_M = 2 * math.pi / 148  # (2 * math.pi) / (152*2)           # Max angle that will be considered for ground lines
T_M_SMALL = 0  # Angle considered to be a small slope
T_B = 0.05  # Max y-intercept for a ground plane line
T_RMSE = 0.2  # Threshold of the Root Mean Square Error of the fit (Recommended: 0.2 - 0.5)
REGRESS_BETWEEN_BINS = True  # Determines if regression for ground lines should occur between two
# neighbouring bins when they're described by different lines
T_D_GROUND = 0.15  # 0.15 # Maximum distance between point and line to be considered part of ground plane
# changed from 0.1
T_D_MAX = 100  # Maximum distance a point can be from the origin to even be considered as
# a ground point. Otherwise it's labelled as a non-ground point.

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
