# The Incremental Algorithm
def get_ground_lines(prototype_points, BIN_COUNT):
    pass


def get_ground_surface(prototype_points, SEGMENT_COUNT, BIN_COUNT):
    # A list of lists that contain ground lines for each segment
    ground_surface = []

    # For every segment
    for i in range(SEGMENT_COUNT):
        # Get list of ground lines that estimate ground surface in segment
        ground_surface.append(get_ground_lines(prototype_points[i], BIN_COUNT))

    return ground_surface

# Notes:
# 1. Ground surface could be a numpy array that is created to be of size
#    SEGMENT_COUNT. Then each entry to be return of get_ground_lines. Then
#    this array can be used in label_points function which is currently the
#    slowest function.
