# Import Custom Modules
from . import point_cloud_processing as pcp

# Import Helper Modules
import numpy as np

# Import Python Modules
import time
import math

# Import Logging
import logging
LOGGER = logging.getLogger(__name__)


def detect_cones(point_cloud, print_logs, LIDAR_RANGE, DELTA_ALPHA, BIN_SIZE, POINT_COUNT, stdout_handler=None):
    # Printing logs to terminal
    if print_logs:
        LOGGER.addHandler(stdout_handler)

    LOGGER.info("Hi from lidar_manager.")

    # Derived Constants
    SEGMENT_COUNT = math.ceil(2 * math.pi / DELTA_ALPHA)
    BIN_COUNT = math.ceil(LIDAR_RANGE / BIN_SIZE)

    start_time = time.time()
    segments_bins_norms = pcp.get_discretised_positions(point_cloud, DELTA_ALPHA, BIN_SIZE)
    end_time = time.time()

    LOGGER.info(f'Numpy PointCloud discretised in {end_time - start_time}s')

    start_time = time.time()
    prototype_points = pcp.get_prototype_points(segments_bins_norms, SEGMENT_COUNT, BIN_COUNT, POINT_COUNT)
    end_time = time.time()

    LOGGER.info(f'Prototype Points computed in {end_time - start_time}s')

    return []
