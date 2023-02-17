from enum import Enum
import math
import pathlib

# Config Parameters
WORKING_DIR: str = str(pathlib.Path(__file__).parent.resolve())
OUTPUT_DIR: str = WORKING_DIR + "/output"
LIBRARY_DIR: str = WORKING_DIR + "/library"
RESOURCES_DIR: str = LIBRARY_DIR + "/resources"
FONTS_DIR: str = RESOURCES_DIR + "/fonts"
MODELS_DIR: str = RESOURCES_DIR + "/models"
QUTMS_LOGO: str = RESOURCES_DIR + "/qutms_logo_white.png"
FIGURES_DIR: str = "/figures"
VIDEOS_DIR: str = "videos"

# Algorithm Parameters | Don't forget algorithm tuning
LIDAR_RANGE = 25  # Max range of points to process (metres)
DELTA_ALPHA = (2 * math.pi) / 128  # Delta angle of segments
BIN_SIZE = 0.14  # Size of bins
T_M = 2 * math.pi / 148  # (2 * math.pi) / (152*2)           # Max angle that will be considered for ground lines
T_M_SMALL = 0  # Angle considered to be a small slope
T_B = 0.05  # Max y-intercept for a ground plane line
T_RMSE = 0.2  # Threshold of the Root Mean Square Error of the fit (Recommended: 0.2 - 0.5)
REGRESS_BETWEEN_BINS = True  # Determines if regression for ground lines should occur between two
# neighbouring bins when they're described by different lines
T_D_GROUND = 0.125  # 0.15 # Maximum distance between point and line to be considered part of ground plane # tune this
# changed from 0.1, ^^ also, the higher this value, the more low object points it will mark as ground BUT this makes dbscan faster
T_D_MAX = 100  # Maximum distance a point can be from the origin to even be considered as
# a ground point. Otherwise it's labelled as a non-ground point.
CPU_UTILISATION = 0.90  # Percentage of CPU Cores to use for multiprocessing ground plane mapping (0.0 - 1.0)
CONE_DIAM = 0.15
CONE_HEIGHT = 0.30

LIDAR_HEIGHT_ABOVE_GROUND = 0.15
LIDAR_VERTICAL_RES = 1.25 * (math.pi / 180)  # 1.25 degrees in between each point
LIDAR_HORIZONTAL_RES = 0.05 * (math.pi / 180)  # NEW
LHAG_ERR = 0.25

HACH_LOWER_ERR = 0.3  # 0.087 - 0.3 < 0 so min bound should probably just be zero lol
HACH_UPPER_ERR = CONE_HEIGHT  # - 0.025

# Derived Parameters
SEGMENT_COUNT = math.ceil(2 * math.pi / DELTA_ALPHA)
BIN_COUNT = math.ceil(LIDAR_RANGE / BIN_SIZE)
HALF_AREA_CONE_HEIGHT = CONE_HEIGHT * (2 - math.sqrt(2)) / 2  # 0.08787

# Expected number of points on a cone at a given distance
NUMER = CONE_HEIGHT * CONE_DIAM
DENOM = 8 * math.tan(LIDAR_VERTICAL_RES / 2) * math.tan(LIDAR_HORIZONTAL_RES / 2)

# Visualiser 1
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

# Visualiser 2
def hex_to_rgb(hex_value: str):
    """Converts HEX to RGB.

    Args:
        hex (str): A colour in HEX format.

    Returns:
        tuple: A colour in RGB format.
    """
    return tuple(int(hex_value.lstrip("#")[i : i + 2], 16) for i in (0, 2, 4))


def rgb_to_normalised_rgba(rgb: tuple):
    """Converts RGB to normalised RGBA.

    Args:
        rgb (tuple): A colour in RGB format.

    Returns:
        tuple: A colour in normalised RGBA format.
    """
    return tuple([c / 255 for c in rgb] + [1])


class Colour(Enum):
    """Enum for HEX colours."""
    MS_ORANGE: str = "#EE7623"
    MS_BLUE: str = "#0F406A"
    DARK_GREY: str = "#1F2630"
    GREY: str = "#555555"
    LIGHT_GREY: str = "#252E3F"
    DIM_BLUE: str = "#225E67"
    BLUE: str = "#9CDCFE"
    LIGHT_BLUE: str = "#43BCCD"
    MINT: str = "#30FDC3"
    GREEN: str = "#23CE6B"
    RED: str = "#B9314F"
    WHITE: str = "#FFFFFF"


class RGBA(Enum):
    """Enum for RGBA colours."""
    MS_ORANGE: tuple = rgb_to_normalised_rgba(hex_to_rgb(Colour.MS_ORANGE.value))
    MS_BLUE: tuple = rgb_to_normalised_rgba(hex_to_rgb(Colour.MS_BLUE.value))
    DARK_GREY: tuple = rgb_to_normalised_rgba(hex_to_rgb(Colour.DARK_GREY.value))
    GREY: tuple = rgb_to_normalised_rgba(hex_to_rgb(Colour.GREY.value))
    LIGHT_GREY: tuple = rgb_to_normalised_rgba(hex_to_rgb(Colour.LIGHT_GREY.value))
    DIM_BLUE: tuple = rgb_to_normalised_rgba(hex_to_rgb(Colour.DIM_BLUE.value))
    BLUE: tuple = rgb_to_normalised_rgba(hex_to_rgb(Colour.BLUE.value))
    LIGHT_BLUE: tuple = rgb_to_normalised_rgba(hex_to_rgb(Colour.LIGHT_BLUE.value))
    MINT: tuple = rgb_to_normalised_rgba(hex_to_rgb(Colour.MINT.value))
    GREEN: tuple = rgb_to_normalised_rgba(hex_to_rgb(Colour.GREEN.value))
    RED: tuple = rgb_to_normalised_rgba(hex_to_rgb(Colour.RED.value))
    WHITE: tuple = rgb_to_normalised_rgba(hex_to_rgb(Colour.WHITE.value))
