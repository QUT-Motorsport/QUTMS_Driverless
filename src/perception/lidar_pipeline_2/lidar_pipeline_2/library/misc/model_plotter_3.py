import numpy as np
import matplotlib.pyplot as plt


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

path = "./src/perception/lidar_pipeline_2/lidar_pipeline_2/library/misc/"
model = "qev3-10.obj"

part_set = []

vertices = []
triangles = []
with open(path + model) as file:
    for line in file.readlines():
        values = line.split()
        if not values:
            continue

        if values[0] == 'v':
            vertices.append(values[1:4])
        elif values[0] == 'f':
            triangles.append(values[1:4])
        #elif values[-1] == 'faces':
        #    break
        #    part_set.append([vertices, triangles])
        #    vertices = []
        #    triangles = []

np_vertices = np.array(vertices, dtype=np.float32)
np_triangles = np.array(triangles, dtype=np.int32) - 1
plt.figure()
ax = plt.axes(projection='3d')
ax.set_title('QEV-3')
ax.plot_trisurf(np_vertices[:, 0], np_vertices[:, 2], np_triangles, np_vertices[:, 1], shade=True, color='grey')
set_axes_equal(ax)
plt.show()