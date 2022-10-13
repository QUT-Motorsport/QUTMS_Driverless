import matplotlib.pyplot as plt
import numpy as np

from .. import utils
from ..constants import Colour


def init_plot_2D(
    title,
    xlabel,
    ylabel,
    background_c=Colour.LIGHT_GREY.value,
    title_c=Colour.BLUE.value,
    face_c=Colour.DARK_GREY.value,
    label_c=Colour.BLUE.value,
    tick_c=Colour.MINT.value,
):

    # Initialise figure
    fig, ax = plt.subplots(facecolor=background_c)

    # Strings
    ax.set_title(title, color=title_c)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)

    # Colours
    ax.set_facecolor(face_c)
    ax.xaxis.label.set_color(label_c)
    ax.yaxis.label.set_color(label_c)
    ax.tick_params(axis="x", colors=tick_c)
    ax.tick_params(axis="y", colors=tick_c)

    # Equal axis
    ax.set_aspect("equal")

    return fig, ax


def add_colourbar(fig, plot, title, title_c, tick_c):
    c_bar = fig.colorbar(plot)
    c_bar.set_label(title, color=title_c, labelpad=10)
    c_bar.ax.yaxis.set_tick_params(color=tick_c)
    plt.setp(plt.getp(c_bar.ax.axes, "yticklabels"), color=tick_c)


def plot_point_cloud_2D(config, point_cloud, name):
    # Create Figure
    fig, ax = init_plot_2D("Point Cloud: " + str(point_cloud.shape[0]) + " Points", "X", "Y")
    plot = ax.scatter(
        point_cloud["x"],
        point_cloud["y"],
        c=point_cloud["intensity"] / 255,
        cmap=plt.cm.gist_rainbow,
        marker="s",
        s=(72.0 / fig.dpi) ** 2,
        vmin=0.0,
        vmax=1.0,
    )

    max_limit = max(np.abs([min(ax.get_xlim()), max(ax.get_xlim()), min(ax.get_ylim()), max(ax.get_ylim())]))
    ax.set_xlim([-max_limit, max_limit])
    ax.set_ylim([-max_limit, max_limit])

    add_colourbar(fig, plot, "Point Intensity", Colour.BLUE.value, Colour.MINT.value)

    # Save Figure
    plt.savefig(f"{config.image_dir}/{name}.png", dpi=225)
