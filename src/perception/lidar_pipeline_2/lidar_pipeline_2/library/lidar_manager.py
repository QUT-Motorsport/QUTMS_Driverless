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


def detect_cones(point_cloud, print_logs, stdout_handler=None):
    # Printing logs to terminal
    if print_logs:
        LOGGER.addHandler(stdout_handler)

    LOGGER.info("Hi from lidar_manager.")

    start_time = time.time()
    pc_discretised = pcp.discretise_point_cloud(point_cloud, 2*math.pi / 128, 0.14)
    end_time = time.time()

    LOGGER.info(f'Numpy PointCloud discretised in {end_time - start_time}s')
    LOGGER.info(pc_discretised)

    return []
