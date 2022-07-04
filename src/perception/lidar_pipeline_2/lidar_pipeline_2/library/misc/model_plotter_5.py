import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np


def set_axes_equal(ax):
    """Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    """

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
    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


path = "./src/perception/lidar_pipeline_2/lidar_pipeline_2/library/misc/"
material = "qev3-10-c.mtl"
model = "qev3-10-c.obj"

materials = dict()
mat_name = None
with open(path + material) as file:
    for line in file.readlines():
        values = line.split()
        if not values:
            continue
        if values[0] == "newmtl":
            mat_name = values[1]
        if values[0] == "Kd":
            materials[mat_name] = tuple([float(values[1]), float(values[2]), float(values[3])])

triangles = []
vertices = []
colours = []
with open(path + model) as file:
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

x = np_vertices[:, 0] * 0.0254
y = np_vertices[:, 2] * 0.0254
z = np_vertices[:, 1] * 0.0254

min_z = np.amin(z)

if min_z > 0:
    z = z - min_z
else:
    z = z + min_z

y_range = np.amax(y) - np.amin(y)

y = y - (y_range / 2)

x_max = np.amax(x)

x = x - x_max

# car is 113.221
# to get x max range of 2.5: divide by 45.2284

triangle_vertices = np.array(
    [
        np.array([[x[T[0]], y[T[0]], z[T[0]]], [x[T[1]], y[T[1]], z[T[1]]], [x[T[2]], y[T[2]], z[T[2]]]])
        for T in np_triangles
    ]
)

collection = Poly3DCollection(triangle_vertices, facecolors=colours, edgecolors=None)

fig = plt.figure()
ax = fig.gca(projection="3d")
ax.add_collection(collection)

ax.set_xlim3d([-70 * 0.0254, 30 * 0.0254])
ax.set_ylim3d([-70 * 0.0254, 30 * 0.0254])
ax.set_zlim3d([-50 * 0.0254, 50 * 0.0254])

plt.show()
