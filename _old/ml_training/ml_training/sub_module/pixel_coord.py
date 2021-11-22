from numpy.lib.function_base import disp
import transforms3d as t3d
import numpy as np
import time
from math import *
import matplotlib.pyplot as plt

from get_yaml import return_track

cones = return_track()

x = list()
y = list()
for cone in cones:
    x.append(cone[0])
    y.append(cone[1])
plt.figure(1)
plt.axis('equal')
plt.plot(x,y)

world_x = [4,0,0]
world_y = [0,0,4]
plt.plot(world_x,world_y, 'r-')

rot_quat = [0.9541231989860535, 0.000429911888204515, -0.0011787827825173736, 0.2994115352630615]

car_coords = [12.43386173248291, 1.153906226158142, 0.24505554139614105]
cam_local = [-0.5, 0, 1, 1] #m

rot_mat = t3d.quaternions.quat2mat(rot_quat) #rad [x, y, z] WRT map

# for determining cam rot
disp_rot = t3d.euler.quat2euler(rot_quat)
rot_points = [4*cos(disp_rot[2]), 4*sin(disp_rot[2])]
rot_points2 = [-4*sin(disp_rot[2]), 4*cos(disp_rot[2])]

car_mat = np.zeros((4,4))
car_mat[0:3, 0:3] = rot_mat
car_mat[3, 0:3] = car_coords
car_mat[3, 3] = 1
print("\ncar_mat: \n", car_mat)

cam_coords = np.matmul(cam_local, car_mat)

# for plotting cam axis
a = cam_coords.tolist()
cam_x = [a[0]+rot_points[0], a[0], a[0]+rot_points2[0]]
cam_y = [a[1]+rot_points[1], a[1], a[1]+rot_points2[1]]
plt.plot(cam_x, cam_y, 'r-')

cam_mat = np.zeros((4,4))
cam_mat[0:3, 0:3] = rot_mat
cam_mat[3, 0:4] = cam_coords
cam_mat[3, 3] = 1
cam_mat = np.asmatrix(cam_mat)
print("\ncam_mat: \n", cam_mat)

cam_mat_i = cam_mat.getI()

cone = cones[0]
print("\ncone: \n",cone)

plt.plot(cone[0], cone[1], 'bo')

cone_on_cam = np.matmul(cone, cam_mat_i)
cone_on_cam = cone_on_cam.tolist()
print("\ncone_on_cam: \n", cone_on_cam[0])

plt.plot(a[0]+cone_on_cam[0][0], a[1]+cone_on_cam[0][1], 'go')


plt.show()


plt.figure(2)
plt.axis('equal')
plt.plot(0, 0, 'bo')
plt.plot(cone_on_cam[0][0], cone_on_cam[0][1], 'go')
plt.show()

x = cone_on_cam[0][2] / -cone_on_cam[0][0]
y = cone_on_cam[0][1] / -cone_on_cam[0][0]
# print(x,y)


