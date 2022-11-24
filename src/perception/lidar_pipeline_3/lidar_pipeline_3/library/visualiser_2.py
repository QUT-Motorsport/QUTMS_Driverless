from PIL import Image
from matplotlib import font_manager
import matplotlib.colors as mpl_colors
import matplotlib.pyplot as plt
import numpy as np

from .. import constants as const
from ..constants import RGBA, Colour

font_dirs = [const.WORKING_DIR + "/library/resources/fonts"]
font_files = font_manager.findSystemFonts(fontpaths=font_dirs)

for font_file in font_files:
    font_manager.fontManager.addfont(font_file)

plt.rcParams["font.family"] = "Roboto"


