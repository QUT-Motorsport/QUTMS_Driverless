# Import Helper Modules
import numpy as np

import time

from numpy.core.fromnumeric import shape


def get_discretised_positions(point_cloud, DELTA_ALPHA, BIN_SIZE):
    # Calculating the segment index for each point
    segments_idx = np.arctan(point_cloud['y'] / point_cloud['x']) / DELTA_ALPHA
    np.nan_to_num(segments_idx, copy=False, nan=((np.pi / 2) / DELTA_ALPHA))  # Limit arctan x->inf = pi/2

    # Calculating the bin index for each point
    point_norms = np.linalg.norm([point_cloud['x'], point_cloud['y']], axis=0)
    bins_idx = point_norms / BIN_SIZE
    np.nan_to_num(bins_idx, copy=False, nan=0)

    return np.column_stack((segments_idx.astype(int, copy=False), bins_idx.astype(int, copy=False), point_norms))


def get_prototype_points(segments_bins_norms, SEGMENT_COUNT, BIN_COUNT, POINT_COUNT):
    pass
    return []
