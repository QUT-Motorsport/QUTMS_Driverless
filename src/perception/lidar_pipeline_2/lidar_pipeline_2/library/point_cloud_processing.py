# Import Helper Modules
import numpy as np

# Import Logging
import logging

from numpy.lib.function_base import copy
LOGGER = logging.getLogger(__name__)


def discretise_point_cloud(point_cloud, DELTA_ALPHA, BIN_SIZE):
    # Calculating the segment index for each point
    segments_idx = np.arctan(point_cloud['y'] / point_cloud['x']) / DELTA_ALPHA
    np.nan_to_num(segments_idx, copy=False, nan=((np.pi / 2) / DELTA_ALPHA))  # Limit arctan x->inf = pi/2

    # Calculating the bin index for each point
    bins_idx = np.linalg.norm([point_cloud['x'], point_cloud['y']], axis=0) / BIN_SIZE
    np.nan_to_num(bins_idx, copy=False, nan=1)

    return [segments_idx, bins_idx]
