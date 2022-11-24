import numpy as np

test = np.array([0, 1, 0, 0, 2, 0, 0, 0, 3, 0, 0, 0, 0, 4, 5, 6, 0, 0, 0])
print("array", test)

non_zeros = np.flatnonzero(test)
print("non zeros", non_zeros)

mask = np.ones(test.size, dtype=bool)
mask[non_zeros] = False
zeros = np.arange(test.size)[mask]

for idx in zeros:
    dists = np.abs(idx - non_zeros)
    wrap_dists = np.abs(test.size - dists)

    min_dist = np.min(dists)
    min_wrap_dist = np.min(wrap_dists)
    if min_dist <= min_wrap_dist:
        closest_idx = non_zeros[np.min(np.where(dists == min_dist))]
    else:
        closest_idx = non_zeros[np.min(np.where(wrap_dists == min_wrap_dist))]

    test[idx] = test[closest_idx]

    print("\ndistances", idx, dists)
    print("closest", closest_idx)
    print("replace", test[closest_idx])
    print("wrap", wrap_dists)

a = np.array(([1, 2, 3], [4, 5, 6]))
print(a[:, 0:2])
