import math
from PIL import Image
from matplotlib import font_manager
import matplotlib.colors as mpl_colors
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
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


def init_plot_2D(title, x_label, y_label):
    # Initialise figure
    fig, ax = plt.subplots(facecolor=RGBA.MS_BLUE.value)

    # Strings
    ax.set_title(title, color=RGBA.MS_ORANGE.value, fontweight="bold", fontsize=14)
    ax.set_xlabel(x_label, fontweight="bold")
    ax.set_ylabel(y_label, fontweight="bold")

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
    ax.view_init(elev=30, azim=202)

    return fig, ax


def add_colourbar(fig, plot, title, title_c, tick_c):
    c_bar = fig.colorbar(plot)
    c_bar.set_label(title, color=title_c, fontweight="bold", labelpad=10)
    c_bar.ax.yaxis.set_tick_params(color=tick_c)
    plt.setp(plt.getp(c_bar.ax.axes, "yticklabels"), color=tick_c)


def add_logo(fig, dpi, small):
    if small:
        logo_size = 0.085
    else:
        logo_size = 0.125

    # Add Logo
    im = Image.open(const.WORKING_DIR + "/library/resources/qutms_logo_white.png")
    #im = Image.open("C:/Users/liamf/Documents/Personal/Software Development/Python/QUTMS_Driverless/src/perception/lidar_pipeline_3/lidar_pipeline_3/library/resources/qutms_logo_white.png")
    im_width, im_height = im.size

    fig_width, fig_height = fig.get_size_inches() * dpi # fig.dpi
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


def plot_point_cloud_2D(config, point_cloud, subtitle, name):
    # Create Figure
    fig, ax = init_plot_2D(f"Point Cloud {subtitle}", "X", "Y")
    plot = ax.scatter(
        point_cloud["x"],
        point_cloud["y"],
        c=point_cloud["intensity"] / 255,
        cmap=plt.cm.gist_rainbow,
        marker="s",
        s=(72.0 / fig.dpi) ** 2,
        linewidths=0,
        vmin=0.0,
        vmax=1.0,
    )

    calibrate_axis(ax)
    add_colourbar(fig, plot, "Point Intensity", RGBA.MS_ORANGE.value, RGBA.WHITE.value)

    ax.text(.01, .99, f'Points: {point_cloud.shape[0]}', ha='left', va='top', c=Colour.WHITE.value, fontsize=8, transform=ax.transAxes)

    # Save Figure
    add_logo(fig, dpi=225, small=True)
    plt.savefig(f"{config.image_dir}/{name}.png", dpi=225)


def plot_cones_3D_2(config, point_cloud, point_labels, cone_centers, name):
    # Create Figure
    fig, ax = init_plot_3D("Cone Locations Identified", "X", "Y", "Z")

    ax.scatter(
        point_cloud["x"],
        point_cloud["y"],
        point_cloud["z"],
        c=point_labels,
        cmap=mpl_colors.ListedColormap([RGBA.GREEN.value, RGBA.RED.value]),
        marker="s",
        s=(72.0 / fig.dpi) ** 2,
        linewidths=0,
        zorder=1,
    )

    # Plot rings around cones
    theta = np.linspace(0, 2 * np.pi, 201)
    x = const.CONE_DIAM * np.cos(theta)
    y = const.CONE_DIAM * np.sin(theta)
    for center in cone_centers:
        ax.plot(
            x + center[0],
            y + center[1],
            -const.LIDAR_HEIGHT_ABOVE_GROUND,
            c=RGBA.WHITE.value,
        )
    
    # Plot car model
    if True:
        ax.add_collection(create_car_model(config))
    
    dist = const.CAR_PLOT_3D_RANGE
    calibrate_axis(ax, [-dist, dist], [-dist, dist], [-dist, dist])

    # Save Figure
    add_logo(fig, dpi=225, small=False)
    plt.savefig(f"{config.image_dir}/{name}.png", dpi=225)


def plot_cones_3D(config, point_cloud, point_labels, cones, name):
    # Create Figure
    fig, ax = init_plot_3D("Cone Locations Identified", "X", "Y", "Z")

    ax.scatter(cones[:, 0], cones[:, 1], cones[:, 2], c=RGBA.WHITE.value, marker="o", zorder=3)
    
    ax.scatter(
        point_cloud["x"],
        point_cloud["y"],
        point_cloud["z"],
        c=point_labels,
        cmap=mpl_colors.ListedColormap([RGBA.GREEN.value, RGBA.RED.value]),
        marker="s",
        s=(72.0 / fig.dpi) ** 2,
        zorder=1,
    )

    calibrate_axis(ax, point_cloud["x"], point_cloud["y"], point_cloud["z"])

    # Save Figure
    add_logo(fig, dpi=225, small=False)
    plt.savefig(f"{config.image_dir}/{name}.png", dpi=225)

    # Create Animation
    if config.animate_figures:
        animate_figure(f"05_{name}_Animated", ax, config.image_dir)


def plot_cones_2D(config, point_cloud, point_labels, cone_centers, cone_points, name):
    fig, ax = init_plot_2D("Cone Locations Identified", "X", "Y")
    ax.scatter(
        point_cloud["x"],
        point_cloud["y"],
        c=point_labels,
        cmap=mpl_colors.ListedColormap([RGBA.GREEN.value, RGBA.RED.value]),
        marker="s",
        s=(72.0 / fig.dpi) ** 2,
        vmin=0.0,
        vmax=1.0,
    )

    ax.scatter(cone_centers[:, 0], cone_centers[:, 1], c=RGBA.WHITE.value, marker=".")

    for i, cone_center in enumerate(cone_centers):
        ax.text(
            cone_center[0] - 0.5,
            cone_center[1] - 1,
            round(np.mean(cone_points[i]["intensity"]), 2),
            c="white",
            fontsize=4,
        )

    calibrate_axis(ax)

    # Save Figure
    add_logo(fig, dpi=225, small=True)
    plt.savefig(f"{config.image_dir}/{name}.png", dpi=225)


def plot_detailed_2D(config, point_cloud, segments, bins, ground_plane, point_labels, reconstructed_objects, reconstructed_centers, cone_intensities, cone_centers, cone_points, duration, name):
    fig, ax = init_plot_2D("LiDAR Perception Pipeline", "X", "Y")

    # Plot Ground Plane
    ground_line_count = 0
    non_empty_segs = np.unique(segments)
    for idx, ground_set in enumerate(ground_plane):
        if ground_set != 0:
            for jdx, ground_line in enumerate(ground_set):
                ground_line_count += 1
                p1 = ground_line[2]
                p2 = ground_line[3]

                x = np.array([p1[0], p2[0]]) * math.cos(const.DELTA_ALPHA * non_empty_segs[idx])
                y = np.array([p1[0], p2[0]]) * math.sin(const.DELTA_ALPHA * non_empty_segs[idx])

                ax.plot(x, y, color=[Colour.LIGHT_BLUE.value, Colour.DIM_BLUE.value][jdx % 2], linewidth=(72.0 / fig.dpi) ** 2, zorder=1)

    # Plot Point Cloud with Labels
    ax.scatter(
        point_cloud["x"][~point_labels],
        point_cloud["y"][~point_labels],
        c=RGBA.GREEN.value,
        marker="s",
        s=(72.0 / fig.dpi) ** 2,
        linewidths=0,
    )
    ax.scatter(reconstructed_centers[:, 0], reconstructed_centers[:, 1], c=RGBA.MS_ORANGE.value, marker='x', s=12, linewidths=2*(72.0 / fig.dpi) ** 2,)
    ax.scatter(
        point_cloud["x"][point_labels],
        point_cloud["y"][point_labels],
        c=RGBA.RED.value,
        marker="s",
        s=(72.0 / fig.dpi) ** 2,
        linewidths=0,
        label='test',
    )

    max_intensity = max([np.max(i['intensity']) for i in cone_points])
    for i, cone_center in enumerate(cone_centers):
        #ax.scatter(cone_center[0], cone_center[1], c=np.mean(cone_points[i]['intensity']) / 255, cmap=plt.cm.gist_rainbow, marker=".")
        ax.scatter(cone_center[0], cone_center[1], c=np.mean(cone_points[i]['intensity'] / max_intensity), cmap=plt.cm.gray_r, marker=".", s=24, linewidths=(72.0 / fig.dpi) ** 2,)
        #ax.text(cone_center[0] - 0.5, cone_center[1] - 1, round(np.mean(cone_points[i]['intensity']), 2), c='white', fontsize=4)
        #ax.text(cone_center[0] - 1.15, cone_center[1] - 0.75, f'({round(cone_center[0], 1):.1f}, {round(cone_center[1], 1):.1f})', c='white', fontsize=3)
        ax.text(cone_center[0] - 1.15, cone_center[1] - 0.75, f'({round(cone_intensities[i], 1):.1f}', c='white', fontsize=3)
    
    for i, rec_center in enumerate(reconstructed_centers):
        ax.text(rec_center[0] - 0.7, rec_center[1] + 0.45, f'{len(reconstructed_objects[i]):<5}', c=Colour.MS_ORANGE.value, fontsize=4)

    ax.text(.01, .99, f'Non Ground Points: {np.count_nonzero(point_labels)}', ha='left', va='top', c=Colour.RED.value, fontsize=8, transform=ax.transAxes)
    ax.text(.01, .95, f'Ground Points: {np.count_nonzero(point_labels == 0)}', ha='left', va='top', c=Colour.GREEN.value, fontsize=8, transform=ax.transAxes)
    ax.text(.01, .91, f'Ground Lines: {ground_line_count}', ha='left', va='top', c=Colour.LIGHT_BLUE.value, fontsize=8, transform=ax.transAxes)
    ax.text(.01, .87, f'Objects: {reconstructed_centers.shape[0]}', ha='left', va='top', c=Colour.MS_ORANGE.value, fontsize=8, transform=ax.transAxes)
    ax.text(.01, .83, f'Cones: {cone_centers.shape[0]}', ha='left', va='top', c=Colour.WHITE.value, fontsize=8, transform=ax.transAxes)
    
    #ax.text(.01, .12, f'Non Empty Segments: {non_empty_segs.shape[0]}', ha='left', va='top', c=Colour.WHITE.value, fontsize=8, transform=ax.transAxes)
    #ax.text(.01, .08, f'Max Bin Assigned: {np.max(bins)}', ha='left', va='top', c=Colour.WHITE.value, fontsize=8, transform=ax.transAxes)
    ax.text(.01, .04, f'Current Hz: {round(1 / duration, 1)}', ha='left', va='top', c=Colour.WHITE.value, fontsize=8, transform=ax.transAxes)
    
    ax.text(1.01, .99, 'Parameters', ha='left', va='top', c=Colour.MS_ORANGE.value, fontsize=6, transform=ax.transAxes)
    ax.text(1.01, .96, f'LIDAR_RANGE = {round(const.LIDAR_RANGE, 4)}', ha='left', va='top', c=Colour.WHITE.value, fontsize=6, transform=ax.transAxes)
    ax.text(1.01, .93, f'DELTA_ALPHA = {round(const.DELTA_ALPHA, 4)}', ha='left', va='top', c=Colour.WHITE.value, fontsize=6, transform=ax.transAxes)
    ax.text(1.01, .90, f'BIN_SIZE = {round(const.BIN_SIZE, 4)}', ha='left', va='top', c=Colour.WHITE.value, fontsize=6, transform=ax.transAxes)
    ax.text(1.01, .87, f'T_M = {round(const.T_M, 4)}', ha='left', va='top', c=Colour.WHITE.value, fontsize=6, transform=ax.transAxes)
    ax.text(1.01, .84, f'T_M_SMALL = {round(const.T_M_SMALL, 4)}', ha='left', va='top', c=Colour.WHITE.value, fontsize=6, transform=ax.transAxes)
    ax.text(1.01, .81, f'T_B = {round(const.T_B, 4)}', ha='left', va='top', c=Colour.WHITE.value, fontsize=6, transform=ax.transAxes)
    ax.text(1.01, .78, f'T_RMSE = {round(const.T_RMSE, 4)}', ha='left', va='top', c=Colour.WHITE.value, fontsize=6, transform=ax.transAxes)
    ax.text(1.01, .75, f'REGRESS_BTWN_BINS = {round(const.REGRESS_BETWEEN_BINS, 4)}', ha='left', va='top', c=Colour.WHITE.value, fontsize=6, transform=ax.transAxes)
    ax.text(1.01, .72, f'T_D_GROUND = {round(const.T_D_GROUND, 4)}', ha='left', va='top', c=Colour.WHITE.value, fontsize=6, transform=ax.transAxes)
    ax.text(1.01, .69, f'CONE_DIAM = {round(const.CONE_DIAM, 4)}', ha='left', va='top', c=Colour.WHITE.value, fontsize=6, transform=ax.transAxes)
    ax.text(1.01, .66, f'CONE_HEIGHT = {round(const.CONE_HEIGHT, 4)}', ha='left', va='top', c=Colour.WHITE.value, fontsize=6, transform=ax.transAxes)
    ax.text(1.01, .63, f'LIDAR_VERT_RES = {round(const.LIDAR_VERTICAL_RES, 4)}', ha='left', va='top', c=Colour.WHITE.value, fontsize=6, transform=ax.transAxes)
    ax.text(1.01, .60, f'LIDAR_HORIZ_RES = {round(const.LIDAR_HORIZONTAL_RES, 4)}', ha='left', va='top', c=Colour.WHITE.value, fontsize=6, transform=ax.transAxes)

    calibrate_axis(ax)

    # Save Figure
    add_logo(fig, dpi=225*2, small=True)
    plt.savefig(f"{config.image_dir}/{name}.png", dpi=225*2)


import os
def create_car_model(config):
    model_path = const.MODEL_DIR

    material_file = None
    object_file = None
    for item in os.listdir(model_path):
        item_split = item.split(".")
        if item_split[-1] == "mtl":
            material_file = item
        elif item_split[-1] == "obj":
            object_file = item

    mat_name = None
    materials = dict()
    with open(model_path + "/" + material_file) as file:
        for line in file.readlines():
            values = line.split()
            if not values:
                continue
            if values[0] == "newmtl":
                mat_name = values[1]
            if values[0] == "Kd":
                materials[mat_name] = tuple([float(values[1]), float(values[2]), float(values[3])])

    colours = []
    vertices = []
    triangles = []
    with open(model_path + "/" + object_file) as file:
        for line in file.readlines():
            values = line.split()
            if not values:
                continue
            if values[0] == "usemtl":
                mat_name = values[1]
            elif values[0] == "v":
                vertices.append(values[1:4])
            elif values[0] == "f":
                triangles.append(values[1:4])
                colours.append(materials[mat_name])

    np_vertices = np.array(vertices, dtype=np.float32)
    np_triangles = np.array(triangles, dtype=np.int32) - 1

    # Convert model from inches to metres
    INCH_TO_M = 0.0254
    x = np_vertices[:, 0] * INCH_TO_M
    y = np_vertices[:, 2] * INCH_TO_M
    z = np_vertices[:, 1] * INCH_TO_M

    # Align bottom of model's wheels with the lowest point in point cloud
    min_z = np.amin(z)
    point_min_z = -const.LIDAR_HEIGHT_ABOVE_GROUND
    z_diff = min_z - point_min_z
    z = z - z_diff

    # Align the front of the model (LiDAR camera location) with origin of point cloud (x=0)
    x_max = np.amax(x)
    x = x - x_max

    # Model vertices and faces
    triangle_vertices = np.array(
        [
            np.array([[x[T[0]], y[T[0]], z[T[0]]], [x[T[1]], y[T[1]], z[T[1]]], [x[T[2]], y[T[2]], z[T[2]]]])
            for T in np_triangles
        ]
    )

    collection = Poly3DCollection(triangle_vertices, facecolors=colours, edgecolors=None)
    return collection
