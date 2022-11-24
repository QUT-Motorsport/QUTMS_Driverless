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


def animate_figure():
    raise NotImplementedError


def init_plot_2D(title, xlabel, ylabel):
    # Initialise figure
    fig, ax = plt.subplots(facecolor=RGBA.MS_BLUE.value)

    # Strings
    ax.set_title(title, color=RGBA.MS_ORANGE.value, fontweight="bold", fontsize=14)
    ax.set_xlabel(xlabel, fontweight="bold")
    ax.set_ylabel(ylabel, fontweight="bold")

    # Colours
    ax.set_facecolor(RGBA.DARK_GREY.value)
    ax.xaxis.label.set_color(RGBA.MS_ORANGE.value)
    ax.yaxis.label.set_color(RGBA.MS_ORANGE.value)
    ax.tick_params(axis="x", colors=RGBA.WHITE.value)
    ax.tick_params(axis="y", colors=RGBA.WHITE.value)

    # Equal axis
    ax.set_aspect("equal")

    return fig, ax


def init_plot_3D(title, xlabel, ylabel, zlabel):
    # Initialise figure
    fig = plt.figure(facecolor=RGBA.MS_BLUE.value)
    ax = fig.add_subplot(projection="3d")
    ax.grid(color="white")

    # Strings
    ax.set_title(title, color=RGBA.MS_ORANGE.value, fontweight="bold", fontsize=14)
    ax.set_xlabel(xlabel, fontweight="bold")
    ax.set_ylabel(ylabel, fontweight="bold")
    ax.set_zlabel(zlabel, fontweight="bold")

    # Colours
    ax.set_facecolor(RGBA.MS_BLUE.value)
    ax.xaxis.set_pane_color(RGBA.DARK_GREY.value)
    ax.yaxis.set_pane_color(RGBA.DARK_GREY.value)
    ax.zaxis.set_pane_color(RGBA.DARK_GREY.value)
    ax.xaxis.label.set_color(RGBA.MS_ORANGE.value)
    ax.yaxis.label.set_color(RGBA.MS_ORANGE.value)
    ax.zaxis.label.set_color(RGBA.MS_ORANGE.value)
    ax.tick_params(axis="x", colors=RGBA.WHITE.value)
    ax.tick_params(axis="y", colors=RGBA.WHITE.value)
    ax.tick_params(axis="z", colors=RGBA.WHITE.value)
    ax.xaxis._axinfo["grid"].update({"color": Colour.MS_BLUE.value})
    ax.yaxis._axinfo["grid"].update({"color": Colour.MS_BLUE.value})
    ax.zaxis._axinfo["grid"].update({"color": Colour.MS_BLUE.value})

    # Equal axis
    ax.set_aspect("equal")

    # Set view anlge
    ax.view_init(elev=34, azim=202)

    return fig, ax


