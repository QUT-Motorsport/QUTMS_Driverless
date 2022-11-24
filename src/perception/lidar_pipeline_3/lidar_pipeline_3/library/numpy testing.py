import numpy as np

import visualiser_2 as vis2

x = np.array([(1.0, 2.0, 3.0), (1.1, 2.2, 3.3), (-1, 27.4, 13.4)], dtype=[("x", "f4"), ("y", "f4"), ("z", "f4")])

xy_cols = x[["x", "y"][:]]

print(xy_cols, xy_cols.shape)

print(np.column_stack((x["x"], x["y"])))

print(x["x"].shape)

print(np.column_stack((x["x"], x["y"])).shape)

a = np.empty((10, 3))
print(type(a))

vis2.plot_cones_3D(None, x, None, None)

import matplotlib.pyplot as plt

plt.show()
