import numpy as np
from numpy.lib.function_base import copy


def test():
    point_cloud = np.array([(1.34255, 2.6543345, 3.65433456), (2.2345, 3.3245, 2.7645), (2.436, 4.73, 5.5)], dtype=[('x', 'float'), ('y', 'float'), ('z', 'float')])
    print(point_cloud[0][1])
    print("wabbles")
    
    print(point_cloud.astype(int, casting='unsafe', copy=False))

    print(np.round(point_cloud[['x', 'y', 'z']], decimals=2))
    print(point_cloud)
    
    yedst = np.linalg.norm([point_cloud['x'], point_cloud['y']], axis=0)
    
    print(yedst)

test()

# # Round Numpy PointCloud to 4 decimal places
        # start_time = time.time()
        # np.round(pc_matrix['x'], decimals=4, out=pc_matrix['x'])
        # np.round(pc_matrix['y'], decimals=4, out=pc_matrix['y'])
        # np.round(pc_matrix['z'], decimals=4, out=pc_matrix['z'])
        # end_time = time.time()

        # LOGGER.info(f'Numpy PointCloud rounded in {end_time - start_time}s')
        # LOGGER.debug(pc_matrix)
