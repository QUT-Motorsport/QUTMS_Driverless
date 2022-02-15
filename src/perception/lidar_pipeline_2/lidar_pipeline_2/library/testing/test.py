import numpy as np

test = np.array([1, 2, 4])
test2 = np.concatenate(([0], test))
print(test2.shape)
