import ground_plane_estimation as gpe
import time
import pickle
from typing import List, NamedTuple, Tuple
from collections import namedtuple

Point = namedtuple('Point', ['x', 'y', 'z', 'intensity', 'ring'])

#path: str = "C:/Users/liamf/OneDrive/QUT/QUTMS/QUTMS_Driverless/src/perception/lidar_pipeline/lidar_pipeline/points_dump/"

#with open(path + "12-07-2021_05-15-02", 'rb') as f:
#    point_cloud = pickle.load(f)

path: str = "C:/Users/liamf/OneDrive/QUT/QUTMS/QUTMS_Driverless/datasets/"
# 24 
with open(path + "08-12-2021_01-55-30.txt", 'rb') as f:
    line: str = f.readline()
    point_cloud: List[namedtuple] = eval(line)

DISPLAY: bool = False
VISUALISE: bool = False
MAX_RANGE: int = 20

start_time = time.time()
gpe.lidar_init(DISPLAY, VISUALISE, "./datasets/figures", MAX_RANGE)
print("test")
gpe.lidar_main(point_cloud)
print("\nTotal Time:", time.time() - start_time)
