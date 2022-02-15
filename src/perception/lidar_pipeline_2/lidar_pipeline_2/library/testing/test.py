import numpy as np

test = np.array([1, 2, 4])
mask = (test == 1) | (test == 4)

print(test[mask])
