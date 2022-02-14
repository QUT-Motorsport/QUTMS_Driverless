import datetime
import matplotlib.pyplot as plt
import numpy as np
import os
import subprocess
import glob
import time


# Default Values
dark_grey = (0.122, 0.149, 0.188, 1.0)
light_grey = (0.145, 0.181, 0.247, 1.0)
blue = (0.612, 0.863, 0.996, 1.0)
mint = (0.188, 0.992, 0.761, 1.0)
default_dir = "/figures"


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
        print("Creating animation frame", str(angle), "/", "360", "|", "{:.2f}%".format(angle / 360 * 100), "|", "{:.0f}m".format(m), "{:.0f}s ".format(s), end = "\r")
        
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


def plot_point_cloud_3D(point_cloud, point_count, working_dir, animate_figures, timestamp):
    # Create Figure
    fig, ax = init_plot_3D('Point Cloud: ' + str(point_count) + ' Points', 'X', 'Y', 'Z')
    plot = ax.scatter(point_cloud['x'], point_cloud['y'], point_cloud['z'], c=point_cloud['intensity']/255, cmap=plt.cm.gist_rainbow, marker='s', s=(72./fig.dpi)**2, vmin=0.0, vmax=1.0)
    add_colourbar(fig, plot, 'Point Intensity', blue, mint)

    # Save Figure
    figure_timestamp = save_figure("02_PointCloud_3D", working_dir, timestamp)
    
    # Create Animation
    if animate_figures:
        animate_figure("01_PointCloud_Animated", ax, figure_timestamp)

def plot_segments_2D(segments_bins_norms_z):
    fig, ax = init_plot_2D("Point Cloud discretised into Segments", "X", "Y")
    pass