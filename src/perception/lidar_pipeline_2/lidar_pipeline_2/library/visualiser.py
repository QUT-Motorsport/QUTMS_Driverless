import datetime
import matplotlib.pyplot as plt
import matplotlib.colors as mpl_colors
import plotly.express.colors as pe_colors
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
    ax.view_init(elev=34, azim=202)

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


def plot_point_cloud_2D(point_cloud, point_count, working_dir, timestamp):
    # Create Figure
    fig, ax = init_plot_2D("Point Cloud: " + str(point_count) + " Points", "X", "Y")
    plot = ax.scatter(point_cloud['x'], point_cloud['y'], c=point_cloud['intensity']/255, cmap=plt.cm.gist_rainbow, marker='s', s=(72./fig.dpi)**2, vmin=0.0, vmax=1.0)
    add_colourbar(fig, plot, 'Point Intensity', blue, mint)

    # Save Figure
    save_figure("01_PointCloud_2D", working_dir, timestamp)


def plot_point_cloud_3D(point_cloud, point_count, working_dir, timestamp, animate_figures):
    # Create Figure
    fig, ax = init_plot_3D('Point Cloud: ' + str(point_count) + ' Points', 'X', 'Y', 'Z')
    plot = ax.scatter(point_cloud['x'], point_cloud['y'], point_cloud['z'], c=point_cloud['intensity']/255, cmap=plt.cm.gist_rainbow, marker='s', s=(72./fig.dpi)**2, vmin=0.0, vmax=1.0)
    add_colourbar(fig, plot, 'Point Intensity', blue, mint)

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


# Notes
# Use a list to store rgba and hex values for colours
# Colour bar legend to have same number of segments with
# repeating colour and add to plots
