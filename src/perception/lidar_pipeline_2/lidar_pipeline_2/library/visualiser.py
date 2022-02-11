import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np


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
    fig = plt.figure(1, figsize=(5, 3), facecolor=background_c)
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
    ax.view_init(elev=34, azim=202.5)

    return fig, ax


def plot_point_cloud(point_cloud):
    colour_map = plt.cm.get_cmap('gist_rainbow')
    fig, ax = init_plot_3D('Point Cloud', 'X', 'Y', 'Z')
    ax.scatter(point_cloud['x'], point_cloud['y'], point_cloud['z'], c=colour_map(point_cloud['intensity']/255), marker='s', s=(72./fig.dpi)**2)
