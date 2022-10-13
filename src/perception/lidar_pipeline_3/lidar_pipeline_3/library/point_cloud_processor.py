import numpy as np

from .. import constants as const


def get_discretised_positions(x, y, point_norms):
    # Calculating the segment index for each point
    segments_idx = np.arctan2(y, x) / const.DELTA_ALPHA
    np.nan_to_num(segments_idx, copy=False, nan=((np.pi / 2) / const.DELTA_ALPHA))  # Limit arctan x->inf = pi/2

    # Calculating the bin index for each point
    bins_idx = point_norms / const.BIN_SIZE

    # Stacking arrays segments_idx, bins_idx, point_norms, and xyz coords into one array
    return segments_idx.astype(int, copy=False), bins_idx.astype(int, copy=False)
