from turtle import back
from matplotlib import projections
import matplotlib.pyplot as plt
import numpy as np


# Scale 255 RGBA values between 0 and 1
def normalise_rgba(rgba):
    return tuple(value / 255 for value in rgba)


def init_plot_3D(title, background_c, title_c, face_c, axis_c, label_c, tick_c):
    fig = plt.figure(facecolor=background_c)
    ax = fig.add_subplot(projection='3d')
    ax.set_title(title, color=title_c)
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


def plot_point_cloud():
    np.random.seed(19680801)

    def randrange(n, vmin, vmax):
        """
        Helper function to make an array of random numbers having shape (n, )
        with each number distributed Uniform(vmin, vmax).
        """
        return (vmax - vmin)*np.random.rand(n) + vmin

    fig = plt.figure(facecolor='#252e3f')
    ax = fig.add_subplot(projection='3d')
    ax.set_title('Point Cloud', color=normalise_rgba((113, 149, 181, 255)))
    ax.set_facecolor('#252e3f')
    ax.w_xaxis.set_pane_color(normalise_rgba((31, 38, 48, 255)))
    ax.w_yaxis.set_pane_color(normalise_rgba((31, 38, 48, 255)))
    ax.w_zaxis.set_pane_color(normalise_rgba((31, 38, 48, 255)))
    ax.xaxis.label.set_color(normalise_rgba((48, 253, 194, 255)))        #setting up X-axis label color to yellow
    ax.yaxis.label.set_color(normalise_rgba((48, 253, 194, 255)))
    ax.zaxis.label.set_color(normalise_rgba((48, 253, 194, 255)))
    ax.tick_params(axis='x', colors=normalise_rgba((48, 253, 194, 255)))    #setting up X-axis tick color to red
    ax.tick_params(axis='y', colors=normalise_rgba((48, 253, 194, 255)))
    ax.tick_params(axis='z', colors=normalise_rgba((48, 253, 194, 255)))
    
    
    #plt.rcParams['ztick.color'] = normalise_rgba((48, 253, 194, 255))


    n = 100

    # For each set of style and range settings, plot n random points in the box
    # defined by x in [23, 32], y in [0, 100], z in [zlow, zhigh].
    for m, zlow, zhigh in [('o', -50, -25), ('^', -30, -5)]:
        xs = randrange(n, 23, 32)
        ys = randrange(n, 0, 100)
        zs = randrange(n, zlow, zhigh)
        ax.scatter(xs, ys, zs, marker=m)

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    plt.show()

plot_point_cloud()