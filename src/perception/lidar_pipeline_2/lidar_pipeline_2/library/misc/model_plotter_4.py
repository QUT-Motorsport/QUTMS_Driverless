import itertools
import matplotlib.pyplot as plt
from pylab import get_cmap
from matplotlib.tri import Triangulation, LinearTriInterpolator
import numpy as np
from scipy import stats
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

path = "./src/perception/lidar_pipeline_2/lidar_pipeline_2/library/misc/"
model = "qev3-10-c.obj"

vertices = []
triangles = []
with open(path + model) as file:
    for line in file.readlines():
        values = line.split()
        if not values:
            continue

        if values[0] == 'usemtl':
            mat_name = values[1]
        if values[0] == 'v':
            vertices.append(values[1:4])
        elif values[0] == 'f':
            triangles.append(values[1:4])

np_vertices = np.array(vertices, dtype=np.float32)
np_triangles = np.array(triangles, dtype=np.float32)

x = np_vertices[:, 0]
y = np_vertices[:, 2]
z = np_vertices[:, 1]

# Creating a triangulation object and using it to extract the actual triangles. 
# Note if it is necessary that no patch will be vertical (i.e. along the z direction)

tri = Triangulation(x, y)

triangle_vertices = np.array([np.array([[x[T[0]], y[T[0]], z[T[0]]],
                                        [x[T[1]], y[T[1]], z[T[1]]], 
                                        [x[T[2]], y[T[2]], z[T[2]]]]) for T in tri.triangles])

# Finding coordinate for the midpoints of each triangle. 
# This will be used to extract the color

midpoints = np.average(triangle_vertices, axis = 1)
midx = midpoints[:, 0]
midy = midpoints[:, 1]

# Interpolating the pdf and using it with the selected cmap to produce the color RGB vector for each face. 
# Some roundoff and normalization are needed

face_color_function = LinearTriInterpolator(tri, z)
face_color_index = face_color_function(midx, midy)
face_color_index[face_color_index < 0] = 0
face_color_index /= np.max(z)

cmap = get_cmap('Spectral')

# Creating the patches and plotting

collection = Poly3DCollection(triangle_vertices)

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.add_collection(collection)
plt.show()