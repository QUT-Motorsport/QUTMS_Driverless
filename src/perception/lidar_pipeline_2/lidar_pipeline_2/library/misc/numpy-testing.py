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

# test_2()


def test_3():
    ground_lines = np.array([0, [[1, 1, [0, 1], [2, 3]], [1, 1, [0, 1], [2, 3]]], [[3, 2, [1, 2], [2, 4]], [3, 2, [1, 2], [2, 4]]], 0, [[5, 6, [1, 0], [3, 0]], [5, 6, [1, 0], [3, 0]]], 0, 0, 0, [[7, 8, [5, 6], [7,8]], [7, 8, [5, 6], [7,8]]], 0, 0], dtype=object)
    
    surface_idx = np.argwhere(ground_lines)

    neighbour_distance = np.arange(0, ground_lines.size) - surface_idx

    np.absolute(neighbour_distance, out=neighbour_distance)

    nearest_idx = np.where(neighbour_distance == neighbour_distance.min(axis=0))
    
    print(surface_idx)

    unique, u_idx = np.unique(nearest_idx[1], return_index=True)

    nearest_surface = nearest_idx[0][u_idx]
    
    print(nearest_surface)

    # New stuff

    test = ground_lines[surface_idx]
    
    test = test.flatten()
    
    view = np.reshape(test.flatten(), (4, 4))
    
    print(view)
    
    
def test4():
    test = np.array((list([1, 2]), list([3, 4])))
    print(type(test))
    print(type(test[0]))
    test = np.array(test)
    print(test.shape)
    print(test)
    print(np.zeros(4))
    
    print(test[:, 0:2])

test4()

def line_to_end_points(line, segment_idx, DELTA_ALPHA):
    start = line[2]  # First point in line
    end = line[3]  # Last point in line

    x_1 = start[0] * math.cos((segment_idx + 0.5) * DELTA_ALPHA)
    x_2 = end[0] * math.cos((segment_idx + 0.5) * DELTA_ALPHA)

    y_1 = start[0] * math.sin((segment_idx + 0.5) * DELTA_ALPHA)
    y_2 = end[0] * math.sin((segment_idx + 0.5) * DELTA_ALPHA)

    p_1 = [x_1, y_1, start[1]]
    p_2 = [x_2, y_2, end[1]]

    return [p_1, p_2]

# # Round Numpy PointCloud to 4 decimal places
# start_time = time.time()
# np.round(pc_matrix['x'], decimals=4, out=pc_matrix['x'])
# np.round(pc_matrix['y'], decimals=4, out=pc_matrix['y'])
# np.round(pc_matrix['z'], decimals=4, out=pc_matrix['z'])
# end_time = time.time()

# LOGGER.info(f'Numpy PointCloud rounded in {end_time - start_time}s')
# LOGGER.debug(pc_matrix)
