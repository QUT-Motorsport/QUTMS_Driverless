import math

import matplotlib.colors as mpl_colors
import matplotlib.pyplot as plt
import numpy as np

from .. import constants as const
from .. import utils


def get_segment_count(segments):
    return abs(segments.min()) + segments.max() + 1


def get_bin_count(bins):
    return bins.max()


def init_plot_2D(
    title,
    xlabel,
    ylabel,
    background_c=const.light_grey,
    title_c=const.blue,
    face_c=const.dark_grey,
    label_c=const.blue,
    tick_c=const.mint,
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


def init_plot_3D(
    title,
    xlabel,
    ylabel,
    zlabel,
    background_c=const.light_grey,
    title_c=const.blue,
    face_c=const.light_grey,
    axis_c=const.dark_grey,
    label_c=const.blue,
    tick_c=const.mint,
):

    # Initialise figure
    fig = plt.figure(facecolor=background_c)
    ax = fig.add_subplot(projection="3d")

    # Strings
    ax.set_title(title, color=title_c)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_zlabel(zlabel)

    # Colours
    ax.set_facecolor(face_c)
    ax.w_xaxis.set_pane_color(axis_c)
    ax.w_yaxis.set_pane_color(axis_c)
    ax.w_zaxis.set_pane_color(axis_c)
    ax.xaxis.label.set_color(label_c)
    ax.yaxis.label.set_color(label_c)
    ax.zaxis.label.set_color(label_c)
    ax.tick_params(axis="x", colors=tick_c)
    ax.tick_params(axis="y", colors=tick_c)
    ax.tick_params(axis="z", colors=tick_c)

    # Equal axis
    ax.set_aspect("equal")

    # Set view anlge
    ax.view_init(elev=34, azim=202)

    return fig, ax


def animate_figure():
    raise NotImplementedError


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

    add_colourbar(fig, plot, "Point Intensity", const.blue, const.mint)

    # Save Figure
    plt.savefig(f"{config.image_dir}/{name}.png", dpi=225)


def plot_segments_2D(config, point_cloud, segments, name):
    fig, ax = init_plot_2D(
        f"Point Cloud Discretised into {abs(segments.min()) + segments.max() + 1} Segments", "X", "Y"
    )
    plot = ax.scatter(
        point_cloud["x"],
        point_cloud["y"],
        c=(segments % len(const.colours_01)),
        cmap=mpl_colors.ListedColormap(const.colours_01),
        marker="s",
        s=(72.0 / fig.dpi) ** 2,
    )

    max_limit = max(np.abs([min(ax.get_xlim()), max(ax.get_xlim()), min(ax.get_ylim()), max(ax.get_ylim())]))
    ax.set_xlim([-max_limit, max_limit])
    ax.set_ylim([-max_limit, max_limit])

    # Save Figure
    plt.savefig(f"{config.image_dir}/{name}.png", dpi=225)


def plot_bins_2D(config, point_cloud, bins, name):
    fig, ax = init_plot_2D(f"Segments Discretised into a Maximum of {bins.max()} Bins", "X", "Y")
    ax.scatter(
        point_cloud["x"],
        point_cloud["y"],
        c=(bins % len(const.colours_01)),
        cmap=mpl_colors.ListedColormap(const.colours_01),
        marker="s",
        s=(72.0 / fig.dpi) ** 2,
    )

    max_limit = max(np.abs([min(ax.get_xlim()), max(ax.get_xlim()), min(ax.get_ylim()), max(ax.get_ylim())]))
    ax.set_xlim([-max_limit, max_limit])
    ax.set_ylim([-max_limit, max_limit])

    # Save Figure
    plt.savefig(f"{config.image_dir}/{name}.png", dpi=225)
