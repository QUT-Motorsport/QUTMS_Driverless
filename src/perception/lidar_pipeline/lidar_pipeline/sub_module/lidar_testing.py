import ground_plane_estimation as gpe
import time
import pickle

with open(f"../cone_dumps/2021-12-07 04:02:51.485253_cones.dump", 'rb') as f:
    cones = pickle.load(f)

start_time = time.time()
gpe.lidar_main(test_data, False, False, "./datasets/figures")
print("\nTotal Time:", time.time() - start_time)