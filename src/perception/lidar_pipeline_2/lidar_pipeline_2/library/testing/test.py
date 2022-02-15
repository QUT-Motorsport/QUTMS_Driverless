import numpy as np

test = np.array([1.5, 2.3, 6.8, 9.2])
print(type(test))

print(test.astype(int, copy=False))

print(np.column_stack((test.astype(int, copy=False), test.astype(int, copy=False))))