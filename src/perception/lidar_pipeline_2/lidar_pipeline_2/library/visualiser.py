import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import os
import subprocess
import glob


# Default Values
dark_grey = (0.122, 0.149, 0.188, 1.0)
light_grey = (0.145, 0.181, 0.247, 1.0)
blue = (0.612, 0.863, 0.996, 1.0)
mint = (0.188, 0.992, 0.761, 1.0)
default_dir = "/figures"


# Scale 255 RGBA values between 0 and 1
def normalise_rgba(rgba):
    return tuple(value / 255 for value in rgba)


def init_plot_3D(title,
                 xlabel,
                 ylabel,
                 zlabel,
                 background_c=light_grey,
                 title_c=blue,
                 face_c=light_grey,
                 axis_c=dark_grey,
                 label_c=mint,
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


def save_figure(name, working_dir, date):
    figures_dir = working_dir + default_dir
    if not os.path.isdir(figures_dir):
        os.mkdir(figures_dir)

    figure_date = figures_dir + date
    if not os.path.isdir(figure_date):
        os.mkdir(figure_date)

    plt.savefig(figure_date + name + ".png", dpi=225)


def animate_figure(ax, working_dir, date):
    animations_folder = working_dir + default_dir + date + '/animations'
    if not os.path.isdir(animations_folder):
        os.mkdir(animations_folder)
    
    for angle in range(360):
        ax.view_init(34, (202 + angle) % 360)
        plt.savefig(animations_folder + "/frame%02d.png" % angle, dpi=225)
        print("Creating frame", str(angle), "/", "360", "|", "{:.2f}%".format(angle / 360 * 100), end = "\r")
        
    os.chdir(animations_folder)
    subprocess.call([
        'ffmpeg', '-framerate', '30', '-i', 'frame%02d.png', '-r', '30', '-pix_fmt', 'yuv420p',
        'video_name.mp4'
    ])
    
    for frames in glob.glob("*.png"):
        os.remove(frames)


def plot_point_cloud(point_cloud, working_dir, date, animate_figures):
    # Create Figure
    fig, ax = init_plot_3D('Point Cloud', 'X', 'Y', 'Z')
    plot = ax.scatter(point_cloud['x'], point_cloud['y'], point_cloud['z'], c=point_cloud['intensity']/255, cmap=plt.cm.gist_rainbow, marker='s', s=(72./fig.dpi)**2, vmin=0.0, vmax=1.0)
    add_colourbar(fig, plot, 'Point Intensity', blue, mint)

    # Save Figure
    save_figure("01-PointCloud", working_dir, date)
    
    # Create Animation
    if animate_figures:
        animate_figure(ax, working_dir, date)
