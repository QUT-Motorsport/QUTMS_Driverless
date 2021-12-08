import ground_plane_estimation as gpe
import time
import pickle
import lidar_pipeline

DISPLAY: bool = True
VISUALISE: bool = True
MAX_RANGE: int = 100

path: str = "C:/Users/liamf/OneDrive/QUT/QUTMS/QUTMS_Driverless/src/perception/lidar_pipeline/lidar_pipeline/points_dump/"

with open(path + "12-07-2021_05-15-02", 'rb') as f:
    point_cloud = pickle.load(f)

start_time = time.time()
gpe.lidar_init(DISPLAY, VISUALISE, "./datasets/figures", MAX_RANGE)
gpe.lidar_main(point_cloud)
print("\nTotal Time:", time.time() - start_time)