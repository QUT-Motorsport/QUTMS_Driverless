import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def plot_model(working_dir, mat_file, obj_file, point_cloud_min_z):
    materials = dict()
    mat_name = None
    with open(working_dir + mat_file) as file:
        for line in file.readlines():
            values = line.split()
            if not values:
                continue
            if values[0] == 'newmtl':
                mat_name = values[1]
            if values[0] == 'Kd':
                materials[mat_name] = tuple([float(values[1]), float(values[2]), float(values[3])])

    colours = []
    vertices = []
    triangles = []
    with open(working_dir + obj_file) as file:
        for line in file.readlines():
            values = line.split()
            if not values:
                continue
            if values[0] == 'usemtl':
                mat_name = values[1]
            elif values[0] == 'v':
                vertices.append(values[1:4])
            elif values[0] == 'f':
                triangles.append(values[1:4])
                colours.append(materials[mat_name])

    np_vertices = np.array(vertices, dtype=np.float32)
    np_triangles = np.array(triangles, dtype=np.float32) - 1

    # Covert model from using freedom units *inches* to cm ... 
    np_vertices = np_vertices * 0.0254
    
    x = np_vertices[:, 0]
    y = np_vertices[:, 2]
    z = np_vertices[:, 1]
    
    min_z = np.amin(z)
    height_diff = min_z - point_cloud_min_z
    if min_z > 0:
        z = z + height_diff
    else:
        z = z - height_diff
    
    # Align front of car model at x = 0 (origin of point cloud)
    x = x - np.amax(x)

    triangle_vertices = np.array([np.array([[x[T[0]], y[T[0]], z[T[0]]],
                                            [x[T[1]], y[T[1]], z[T[1]]], 
                                            [x[T[2]], y[T[2]], z[T[2]]]]) for T in np_triangles])
    
    collection = Poly3DCollection(triangle_vertices, facecolors=colours, edgecolors=None)
    return collection
