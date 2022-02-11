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
    ax.view_init(elev=34, azim=202.5)

    return fig, ax


def plot_point_cloud(point_cloud):
    fig, ax = init_plot_3D('Point Cloud', 'X', 'Y', 'Z')
    plot = ax.scatter(point_cloud['x'], point_cloud['y'], point_cloud['z'], c=point_cloud['intensity']/255, cmap=plt.cm.gist_rainbow, marker='s', s=(72./fig.dpi)**2, vmin=0.0, vmax=1.0)
    c_bar = fig.colorbar(plot)
    
    c_bar.set_label('Intensity', color=normalise_rgba((48, 253, 194, 255)))
    c_bar.ax.yaxis.set_tick_params(color=normalise_rgba((48, 253, 194, 255)))
    c_bar.outline.set_edgecolor(normalise_rgba((31, 38, 48, 255)))
    plt.setp(plt.getp(c_bar.ax.axes, 'yticklabels'), color=normalise_rgba((48, 253, 194, 255)))
    
    #for angle in range(0, 360):
    #    ax.view_init(34, angle)
    #    plt.draw()
    #    plt.pause(.00001)
    #    plt.savefig("./test.png")
