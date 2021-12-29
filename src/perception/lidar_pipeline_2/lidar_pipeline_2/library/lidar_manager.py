# Import Custom Modules
from . import point_cloud_processing as pcp
from . import ground_plane_estimation as gpe

# Import Python Modules
import time
import math

# Import Logging
import logging
LOGGER = logging.getLogger(__name__)


def detect_cones(
        point_cloud,
        point_norms,
        print_logs,
        LIDAR_RANGE,
        DELTA_ALPHA,
        BIN_SIZE,
        T_M,
        T_M_SMALL,
        T_B,
        T_RMSE,
        REGRESS_BETWEEN_BINS,
        point_count,
        stdout_handler):

    # Printing logs to terminal
    if print_logs:
        LOGGER.addHandler(stdout_handler)

    LOGGER.info("Hi from lidar_manager.")

    # Derived Constants
    SEGMENT_COUNT = math.ceil(2 * math.pi / DELTA_ALPHA)
    BIN_COUNT = math.ceil(LIDAR_RANGE / BIN_SIZE)

    # Discretise point cloud for real-time performance
    start_time = time.time()
    segments_bins_norms_z = pcp.get_discretised_positions_2(point_cloud, point_norms, DELTA_ALPHA, BIN_SIZE)
    end_time = time.time()

    LOGGER.info(f'Numpy PointCloud discretised in {end_time - start_time}s')

    # Calculate prototype point for every bin (if one exists)
    start_time = time.time()
    prototype_points = pcp.get_prototype_points_2(segments_bins_norms_z)
    end_time = time.time()

    LOGGER.info(f'Prototype Points computed in {end_time - start_time}s')

    start_time = time.time()
    ground_surface = gpe.get_ground_surface_2(prototype_points, SEGMENT_COUNT, BIN_COUNT, T_M, T_M_SMALL, T_B, T_RMSE, REGRESS_BETWEEN_BINS)
    end_time = time.time()

    LOGGER.info(f'Ground Surface estimated in {end_time - start_time}s')

    return []
