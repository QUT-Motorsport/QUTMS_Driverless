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
    fig, ax = init_plot_2D(f"Point Cloud Discretised into {get_segment_count(segments)} Segments", "X", "Y")
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


def plot_segments_3D(config, point_cloud, segments, name):
    fig, ax = init_plot_3D(f"Point Cloud Discretised into {get_segment_count(segments)} Segments", "X", "Y", "Z")

    ax.scatter(
        point_cloud["x"],
        point_cloud["y"],
        point_cloud["z"],
        c=(segments % len(const.colours_01)),
        cmap=mpl_colors.ListedColormap(const.colours_01),
        marker="s",
        s=(72.0 / fig.dpi) ** 2,
    )

    max_limit = max(
        np.abs(
            [
                min(point_cloud["x"]),
                max(point_cloud["x"]),
                min(point_cloud["y"]),
                max(point_cloud["y"]),
                min(point_cloud["z"]),
                max(point_cloud["z"]),
            ]
        )
    )
    ax.axes.set_xlim(-max_limit, max_limit)
    ax.axes.set_ylim(-max_limit, max_limit)
    ax.axes.set_zlim(-max_limit, max_limit)

    # Save Figure
    plt.savefig(f"{config.image_dir}/{name}.png", dpi=225)

    # Create Animation
    if config.animate_figures:
        animate_figure(f"02_{name}_Animated", ax, config.image_dir)


def plot_bins_3D(config, point_cloud, bins, name):
    fig, ax = init_plot_3D(f"Segments Discretised into a Maximum of {get_bin_count(bins)} Bins", "X", "Y", "Z")

    ax.scatter(
        point_cloud["x"],
        point_cloud["y"],
        point_cloud["z"],
        c=(bins % len(const.colours_01)),
        cmap=mpl_colors.ListedColormap(const.colours_01),
        marker="s",
        s=(72.0 / fig.dpi) ** 2,
    )

    max_limit = max(
        np.abs(
            [
                min(point_cloud["x"]),
                max(point_cloud["x"]),
                min(point_cloud["y"]),
                max(point_cloud["y"]),
                min(point_cloud["z"]),
                max(point_cloud["z"]),
            ]
        )
    )
    ax.axes.set_xlim(-max_limit, max_limit)
    ax.axes.set_ylim(-max_limit, max_limit)
    ax.axes.set_zlim(-max_limit, max_limit)

    # Save Figure
    plt.savefig(f"{config.image_dir}/{name}.png", dpi=225)

    # Create Animation
    if config.animate_figures:
        animate_figure(f"03_{name}_Animated", ax, config.image_dir)


def plot_prototype_points_2D(config, proto_segs_arr, proto_segs, name):
    fig, ax = init_plot_2D("Prototype Points", "X", "Y")

    for idx, segment in enumerate(proto_segs_arr):
        segment_num = proto_segs[idx]
        x = segment[:, 0] * math.cos(const.DELTA_ALPHA * segment_num)
        y = segment[:, 0] * math.sin(const.DELTA_ALPHA * segment_num)

        ax.scatter(
            x, y, c=const.colours_01[proto_segs[idx] % len(const.colours_01)], marker="s", s=(72.0 / fig.dpi) ** 2
        )

    max_limit = max(np.abs([min(ax.get_xlim()), max(ax.get_xlim()), min(ax.get_ylim()), max(ax.get_ylim())]))
    ax.set_xlim([-max_limit, max_limit])
    ax.set_ylim([-max_limit, max_limit])

    # Save Figure
    plt.savefig(f"{config.image_dir}/{name}.png", dpi=225)


def plot_prototype_points_3D(config, proto_segs_arr, proto_segs, name):
    fig, ax = init_plot_3D("Prototype Points", "X", "Y", "Z")

    for idx, segment in enumerate(proto_segs_arr):
        segment_num = proto_segs[idx]
        x = segment[:, 0] * math.cos(const.DELTA_ALPHA * segment_num)
        y = segment[:, 0] * math.sin(const.DELTA_ALPHA * segment_num)

        ax.scatter(
            x,
            y,
            segment[:, 1],
            c=const.colours_01[proto_segs[idx] % len(const.colours_01)],
            marker="s",
            s=(72.0 / fig.dpi) ** 2,
        )

    max_limit = max(np.abs([min(x), max(x), min(y), max(y), min(segment[:, 1]), max(segment[:, 1])]))
    ax.axes.set_xlim(-max_limit, max_limit)
    ax.axes.set_ylim(-max_limit, max_limit)
    ax.axes.set_zlim(-max_limit, max_limit)

    # Save Figure
    plt.savefig(f"{config.image_dir}/{name}.png", dpi=225)

    # Create Animation
    if config.animate_figures:
        animate_figure(f"04_{name}_Animated", ax, config.image_dir)


def plot_ground_plane_2D(config, ground_plane, proto_segs_arr, proto_segs, name):
    fig, ax = init_plot_2D("Ground Plane Fitted", "X", "Y")

    # Plot Prototype Points
    for idx, segment in enumerate(proto_segs_arr):
        segment_num = proto_segs[idx]
        x = segment[:, 0] * math.cos(const.DELTA_ALPHA * segment_num)
        y = segment[:, 0] * math.sin(const.DELTA_ALPHA * segment_num)

        ax.scatter(x, y, c=const.mint_hex, marker="s", s=(72.0 / fig.dpi) ** 2)

    # Plot Ground Plane
    for idx, ground_set in enumerate(ground_plane):
        if ground_set != 0:
            for jdx, ground_line in enumerate(ground_set):
                p1 = ground_line[2]
                p2 = ground_line[3]

                x = np.array([p1[0], p2[0]]) * math.cos(const.DELTA_ALPHA * idx)
                y = np.array([p1[0], p2[0]]) * math.sin(const.DELTA_ALPHA * idx)

                ax.plot(x, y, color=const.colours_01[jdx % len(const.colours_01)])

    max_limit = max(np.abs([min(ax.get_xlim()), max(ax.get_xlim()), min(ax.get_ylim()), max(ax.get_ylim())]))
    ax.set_xlim([-max_limit, max_limit])
    ax.set_ylim([-max_limit, max_limit])

    # Save Figure
    plt.savefig(f"{config.image_dir}/{name}.png", dpi=225)


def plot_ground_plane_3D(config, ground_plane, proto_segs_arr, proto_segs, name):
    fig, ax = init_plot_3D("Ground Plane Fitted", "X", "Y", "Z")

    for idx, segment in enumerate(proto_segs_arr):
        segment_num = proto_segs[idx]
        x = segment[:, 0] * math.cos(const.DELTA_ALPHA * segment_num)
        y = segment[:, 0] * math.sin(const.DELTA_ALPHA * segment_num)

        ax.scatter(x, y, segment[:, 1], color=const.mint_hex, marker="s", s=(72.0 / fig.dpi) ** 2)

    # Plot Ground Plane
    for idx, ground_set in enumerate(ground_plane):
        if ground_set != 0:
            for jdx, ground_line in enumerate(ground_set):
                p1 = ground_line[2]
                p2 = ground_line[3]

                x = np.array([p1[0], p2[0]]) * math.cos(const.DELTA_ALPHA * idx)
                y = np.array([p1[0], p2[0]]) * math.sin(const.DELTA_ALPHA * idx)
                z = np.array([p1[1], p2[1]])

                ax.plot(x, y, z, color=const.colours_01[jdx % len(const.colours_01)])

    max_limit = max(np.abs([min(x), max(x), min(y), max(y), min(segment[:, 1]), max(segment[:, 1])]))
    ax.axes.set_xlim(-max_limit, max_limit)
    ax.axes.set_ylim(-max_limit, max_limit)
    ax.axes.set_zlim(-max_limit, max_limit)

    # Save Figure
    plt.savefig(f"{config.image_dir}/{name}.png", dpi=225)

    # Create Animation
    if config.animate_figures:
        animate_figure(f"05_{name}_Animated", ax, config.image_dir)
