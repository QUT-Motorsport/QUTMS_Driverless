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


def add_logo(fig, dpi, small):
    if small:
        logo_size = 0.1
    else:
        logo_size = 0.125

    # Add Logo
    im = Image.open(const.WORKING_DIR + "/library/resources/qutms_logo_white.png")
    im_width, im_height = im.size

    fig_width, fig_height = fig.get_size_inches() * 225  # fig.dpi
    new_height = int(fig_height * logo_size)
    im = im.resize((int(new_height * (im_width / im_height)), new_height))
    fig.figimage(im, 10, 10, alpha=0.75, origin="upper", zorder=3)


def calibrate_axis(ax, x=None, y=None, z=None):
    if x is not None:
        max_limit = max(np.abs([np.min(x), np.max(x), np.min(y), np.max(y), np.min(z), np.max(z)]))
        ax.axes.set_zlim(-max_limit, max_limit)
    else:
        max_limit = max(np.abs([min(ax.get_xlim()), max(ax.get_xlim()), min(ax.get_ylim()), max(ax.get_ylim())]))
    ax.axes.set_xlim(-max_limit, max_limit)
    ax.axes.set_ylim(-max_limit, max_limit)


