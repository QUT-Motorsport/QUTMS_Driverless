import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import os
import subprocess
import glob


# Scale 255 RGBA values between 0 and 1
def normalise_rgba(rgba):
    return tuple(value / 255 for value in rgba)


def init_plot_3D(title,
                 xlabel,
                 ylabel,
                 zlabel,
                 background_c=normalise_rgba((37, 46, 63, 255)),
                 title_c=normalise_rgba((156, 220, 254, 255)),
                 face_c=normalise_rgba((37, 46, 63, 255)),
                 axis_c=normalise_rgba((31, 38, 48, 255)),
                 label_c=normalise_rgba((48, 253, 194, 255)),
                 tick_c=normalise_rgba((48, 253, 194, 255))):
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


def generate_video(ax, working_dir):
    print("Hello?")
    animations_folder = working_dir + '/animations'
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
    
    for file_name in glob.glob("*.png"):
        os.remove(file_name)
    


def plot_point_cloud(point_cloud, working_dir):
    fig, ax = init_plot_3D('Point Cloud', 'X', 'Y', 'Z')
    plot = ax.scatter(point_cloud['x'], point_cloud['y'], point_cloud['z'], c=point_cloud['intensity']/255, cmap=plt.cm.gist_rainbow, marker='s', s=(72./fig.dpi)**2, vmin=0.0, vmax=1.0)
    c_bar = fig.colorbar(plot)
    
    c_bar.set_label('Point Intensity', color=normalise_rgba((156, 220, 254, 255)), labelpad=10)
    c_bar.ax.yaxis.set_tick_params(color=normalise_rgba((156, 220, 254, 255)))
    plt.setp(plt.getp(c_bar.ax.axes, 'yticklabels'), color=normalise_rgba((156, 220, 254, 255)))
    
    generate_video(ax, working_dir)
