import numpy as np
from numpy.lib.function_base import copy


def test():
    point_cloud = np.array([(2.2, 3.3, 1.1), (1.1, 1.1, 2.2), (3.3, 2.2, 3.3)], dtype=[('x', 'float'), ('y', 'float'), ('z', 'float')])
    print(point_cloud[[0, 1, 2]])
    a = np.array([[1, 2], [3, 4]])
    b = np.array([[5, 6]])
    a = np.concatenate((a, b), axis=0)
    
    hi = np.array([[2, 9], [2, 9], [3, 5], [2, 7], [3, 3], [2, 7]])
    print(hi)
    un, idx = np.unique(hi, return_index=True, axis=0)
    print('YAS', un)
    print('YAS', idx)
    # print('YAS', idx)
    # print('NAN', hi[idx])
    
    print(a)

def test_2():
    ground_lines = np.array([0, [1, 2], [3, 4], 0, [5, 6], 0, 0, 0, [7, 8], 0, 0], dtype=object)
    
    idx = np.argwhere(ground_lines)
    
    test = np.arange(0, ground_lines.size)
    
    test2 = test - idx
    
    np.absolute(test2, out=test2)
    
    test3 = test2.min(axis=0)
    
    print(idx, test)
    
    print(test2)
    
    print(test3)
    
    test4 = np.where(test2 == test3)
    
    print(test4)
    
    unique, u_idx = np.unique(test4[1], return_index=True)
    
    print(test4[0][u_idx])

test_2()

# # Round Numpy PointCloud to 4 decimal places
# start_time = time.time()
# np.round(pc_matrix['x'], decimals=4, out=pc_matrix['x'])
# np.round(pc_matrix['y'], decimals=4, out=pc_matrix['y'])
# np.round(pc_matrix['z'], decimals=4, out=pc_matrix['z'])
# end_time = time.time()

# LOGGER.info(f'Numpy PointCloud rounded in {end_time - start_time}s')
# LOGGER.debug(pc_matrix)
