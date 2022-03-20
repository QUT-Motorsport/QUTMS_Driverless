import datetime
import matplotlib.pyplot as plt
import matplotlib.colors as mpl_colors
import numpy as np
import os
import subprocess
import glob
import time
import math


# Default Values
dark_grey = (0.122, 0.149, 0.188, 1.0) #1f2630
light_grey = (0.145, 0.181, 0.247, 1.0) #252e3f
blue = (0.612, 0.863, 0.996, 1.0) #9cdcfe
mint = (0.188, 0.992, 0.761, 1.0) #30fdc3
default_dir = "/figures"

# Custom Colours
mint_hex = '#30fdc3'
yellow_hex = '#F4D44D'
red_hex = '#F45060'
blue_hex = '#636EFA'

# Colour Sets
colours_01 = [mint_hex, yellow_hex, red_hex, blue_hex]

# Scale 255 RGBA values between 0 and 1
def normalise_rgba(rgba):
    return tuple(value / 255 for value in rgba)


def init_plot_2D(title,
                 xlabel,
                 ylabel,
                 background_c=light_grey,
                 title_c=blue,
                 face_c=dark_grey,
                 label_c=blue,
                 tick_c=mint):

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
    ax.tick_params(axis='x', colors=tick_c)
    ax.tick_params(axis='y', colors=tick_c)

    return fig, ax


def init_plot_3D(title,
                 xlabel,
                 ylabel,
                 zlabel,
                 background_c=light_grey,
                 title_c=blue,
                 face_c=light_grey,
                 axis_c=dark_grey,
                 label_c=blue,
                 tick_c=mint):

    # Initialise figure
    fig = plt.figure(facecolor=background_c)
    ax = fig.add_subplot(projection='3d')

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
    ax.tick_params(axis='x', colors=tick_c)
    ax.tick_params(axis='y', colors=tick_c)
    ax.tick_params(axis='z', colors=tick_c)

    # Set view anlge
    ax.view_init(elev=0, azim=90)

    return fig, ax


def add_colourbar(fig, plot, title, title_c, tick_c):
    c_bar = fig.colorbar(plot)
    c_bar.set_label(title, color=title_c, labelpad=10)
    c_bar.ax.yaxis.set_tick_params(color=tick_c)
    plt.setp(plt.getp(c_bar.ax.axes, 'yticklabels'), color=tick_c)


def save_figure(name, working_dir, timestamp):
    figures_dir = working_dir + default_dir
    if not os.path.isdir(figures_dir):
        os.mkdir(figures_dir)

    figure_timestamp = figures_dir + "/" + timestamp
    if not os.path.isdir(figure_timestamp):
        os.mkdir(figure_timestamp)

    plt.savefig(figure_timestamp + "/" + name + ".png", dpi=225)

    return figure_timestamp


def animate_figure(name, ax, figure_timestamp):
    animations_folder = figure_timestamp + '/animations'
    if not os.path.isdir(animations_folder):
        os.mkdir(animations_folder)

    print(f"Creating animation {name} ...")

    average_time = 0
    for angle in range(360):
        start_time = time.time()
        ax.view_init(34, (202 + angle) % 360)
        plt.savefig(animations_folder + "/frame%02d.png" % angle, dpi=225)
        total_time = time.time() - start_time

        if average_time == 0:
            average_time = total_time
        else:
            average_time = (average_time + total_time) / 2

        m, s = divmod(average_time * (359 - angle), 60)
        print("Animating frame", str(angle), "/", "360", "|", "{:.2f}%".format(angle / 360 * 100), "|", "{:.0f}m".format(m), "{:.0f}s ".format(s), end = "\r")
        
    os.chdir(animations_folder)
    subprocess.call([
        'ffmpeg', '-framerate', '30', '-i', 'frame%02d.png', '-r', '30', '-pix_fmt', 'yuv420p',
        name + '.mp4'
    ])
    
    for frames in glob.glob("*.png"):
        os.remove(frames)


def create_car_model(point_cloud, working_dir):
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    
    model_path = "/model"

    material_file = None
    object_file = None
    for item in os.listdir(working_dir + model_path):
        item_split = item.split('.')
        if item_split[-1] == 'mtl':
            material_file = item
        elif item_split[-1] == 'obj':
            object_file = item

    mat_name = None
    materials = dict()
    with open(working_dir + model_path + '/' + material_file) as file:
        for line in file.readlines():
            values = line.split()
            if not values:
                continue
            if values[0] == 'newmtl':
                mat_name = values[1]
            if values[0] == 'Kd':
                materials[mat_name] = tuple([float(values[1]), float(values[2]), float(values[3])])

    colours = []
    vertices = []
    triangles = []
    with open(working_dir + model_path + '/' + object_file) as file:
        for line in file.readlines():
            values = line.split()
            if not values:
                continue
            if values[0] == 'usemtl':
                mat_name = values[1]
            elif values[0] == 'v':
                vertices.append(values[1:4])
            elif values[0] == 'f':
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
    point_min_z = np.amin(point_cloud['z'])
    z_diff = min_z - point_min_z
    z = z - z_diff

    # Align the front of the model (LiDAR camera location) with origin of point cloud (x=0)
    x_max = np.amax(x)
    x = x - x_max

    # Model vertices and faces
    triangle_vertices = np.array([np.array([[x[T[0]], y[T[0]], z[T[0]]],
                                            [x[T[1]], y[T[1]], z[T[1]]], 
                                            [x[T[2]], y[T[2]], z[T[2]]]]) for T in np_triangles])

    collection = Poly3DCollection(triangle_vertices, facecolors=colours, edgecolors=None)
    return collection

def plot_point_cloud_2D(point_cloud, point_count, working_dir, timestamp):
    # Create Figure
    fig, ax = init_plot_2D("Point Cloud: " + str(point_count) + " Points", "X", "Y")
    plot = ax.scatter(point_cloud['x'], point_cloud['y'], c=point_cloud['intensity']/255, cmap=plt.cm.gist_rainbow, marker='s', s=(72./fig.dpi)**2, vmin=0.0, vmax=1.0)
    add_colourbar(fig, plot, 'Point Intensity', blue, mint)

    # Save Figure
    save_figure("01_PointCloud_2D", working_dir, timestamp)


def plot_point_cloud_3D(point_cloud, point_count, working_dir, timestamp, animate_figures, model_car):
    # Create Figure
    fig, ax = init_plot_3D('Point Cloud: ' + str(point_count) + ' Points', 'X', 'Y', 'Z')
    plot = ax.scatter(point_cloud['x'], point_cloud['y'], point_cloud['z'], c=point_cloud['intensity']/255, cmap=plt.cm.gist_rainbow, marker='s', s=(72./fig.dpi)**2, vmin=0.0, vmax=1.0)
    add_colourbar(fig, plot, 'Point Intensity', blue, mint)

    # Plot car model
    if model_car:
        ax.add_collection(create_car_model(point_cloud, working_dir))

    x_max = np.amax(point_cloud['x'])
    y_max = np.amax(point_cloud['y'])
    z_max = np.amax(point_cloud['z'])
    max_val = max(x_max, y_max, z_max)

    ax.set_xlim3d([-max_val, max_val])
    ax.set_ylim3d([-max_val, max_val])
    ax.set_zlim3d([-max_val, max_val])

    # Save Figure
    figure_timestamp = save_figure("02_PointCloud_3D", working_dir, timestamp)
    
    # Create Animation
    if animate_figures:
        animate_figure("01_PointCloud_Animated", ax, figure_timestamp)


def get_segment_count(segments):
    return (abs(segments.min()) + segments.max() + 1)


def plot_segments_2D(point_cloud, segments, working_dir, timestamp):
    fig, ax = init_plot_2D(f"Point Cloud Discretised into {get_segment_count(segments)} Segments", "X", "Y")
    plot = ax.scatter(point_cloud['x'], point_cloud['y'], c=(segments % len(colours_01)), cmap=mpl_colors.ListedColormap(colours_01), marker='s', s=(72./fig.dpi)**2)

    # Save Figure
    save_figure("03_PointCloudSegments_2D", working_dir, timestamp)


def plot_segments_3D(point_cloud, segments, working_dir, timestamp, animate_figures):
    fig, ax = init_plot_3D(f"Point Cloud Discretised into {get_segment_count(segments)} Segments", "X", "Y", "Z")

    ax.scatter(point_cloud['x'], point_cloud['y'], point_cloud['z'], c=(segments % len(colours_01)), cmap=mpl_colors.ListedColormap(colours_01), marker='s', s=(72./fig.dpi)**2)

    # Save Figure
    figure_timestamp = save_figure("04_PointCloudSegments_3D", working_dir, timestamp)
    
    # Create Animation
    if animate_figures:
        animate_figure("02_PointCloudSegments_Animated", ax, figure_timestamp)


def get_bin_count(bins):
    return bins.max()

def plot_bins_2D(point_cloud, bins, working_dir, timestamp):
    fig, ax = init_plot_2D(f"Segments Sliced into a Maximum of {get_bin_count(bins)} Bins", "X", "Y")
    ax.scatter(point_cloud['x'], point_cloud['y'], c=(bins % len(colours_01)), cmap=mpl_colors.ListedColormap(colours_01), marker='s', s=(72./fig.dpi)**2)

    # Save Figure
    save_figure("05_PointCloudBins_2D", working_dir, timestamp)


def plot_bins_3D(point_cloud, bins, working_dir, timestamp, animate_figures):
    fig, ax = init_plot_3D(f"Segments Sliced into a Maximum of {get_bin_count(bins)} Bins", "X", "Y", "Z")

    ax.scatter(point_cloud['x'], point_cloud['y'], point_cloud['z'], c=(bins % len(colours_01)), cmap=mpl_colors.ListedColormap(colours_01), marker='s', s=(72./fig.dpi)**2)

    # Save Figure
    figure_timestamp = save_figure("06_PointCloudBins_3D", working_dir, timestamp)

    # Create Animation
    if animate_figures:
        animate_figure("03_PointCloudBins_Animated", ax, figure_timestamp)


def plot_prototype_points_2D(split_prototype_segments, prototype_segments, DELTA_ALPHA, working_dir, timestamp):
    fig, ax = init_plot_2D("Prototype Points", "X", "Y")

    for idx, segment in enumerate(split_prototype_segments):
        segment_num = prototype_segments[idx]
        x = segment[:, 0] * math.cos(DELTA_ALPHA * segment_num)
        y = segment[:, 0] * math.sin(DELTA_ALPHA * segment_num)

        ax.scatter(x, y, c=colours_01[prototype_segments[idx] % len(colours_01)], marker='s', s=(72./fig.dpi)**2)

    # Save Figure
    save_figure("07_PrototypePoints_2D", working_dir, timestamp)


def plot_prototype_points_3D(split_prototype_segments, prototype_segments, DELTA_ALPHA, working_dir, timestamp, animate_figures):
    fig, ax = init_plot_3D("Prototype Points", "X", "Y", "Z")

    for idx, segment in enumerate(split_prototype_segments):
        segment_num = prototype_segments[idx]
        x = segment[:, 0] * math.cos(DELTA_ALPHA * segment_num)
        y = segment[:, 0] * math.sin(DELTA_ALPHA * segment_num)
        
        ax.scatter(x, y, segment[:, 1], c=colours_01[prototype_segments[idx] % len(colours_01)], marker='s', s=(72./fig.dpi)**2)

    # Save Figure
    figure_timestamp = save_figure("08_PrototypePoints_3D", working_dir, timestamp)

    # Create Animation
    if animate_figures:
        animate_figure("04_PrototypePoints_Animated", ax, figure_timestamp)


def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

# Notes
# Use a list to store rgba and hex values for colours
# Colour bar legend to have same number of segments with
# repeating colour and add to plots
