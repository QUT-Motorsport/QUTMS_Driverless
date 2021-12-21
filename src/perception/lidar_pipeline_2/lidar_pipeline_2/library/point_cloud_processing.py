# Import Helper Modules
import numpy as np

# Import Python Modules
import math

# Import Logging
import logging
LOGGER = logging.getLogger(__name__)


def discretise_point_cloud(point_cloud, DELTA_ALPHA, BIN_SIZE, SEGMENT_COUNT, BIN_COUNT):
    # Calculating the segment index for each point
    segments_idx = np.arctan(point_cloud['y'] / point_cloud['x']) // DELTA_ALPHA

    # Calculating the bin index for each point
    bins_idx = np.linalg.norm([point_cloud['x'], point_cloud['y']], axis=0) // BIN_SIZE

    segments_bins = np.empty((SEGMENT_COUNT, BIN_COUNT, ))

    return segments_idx
