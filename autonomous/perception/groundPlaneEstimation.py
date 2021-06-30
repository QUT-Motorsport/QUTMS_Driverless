import math

# many points [x, y, z]
test_data = [[0, 0, 1], [0, 1, 0], [0, 1, 1], [1, 0, 0], [1, 0, 1], [1, 1, 0], [1, 1, 1]]

LIDAR_RANGE = 10 # the max range of the LiDAR 

DELTA_ALPHA = math.pi / 4 # the angle every segment covers (45deg)
M = 2*math.pi / DELTA_ALPHA # number of segments

# Returns the index to a segment that a point maps to
def get_segment(x, y):
    return math.atan2(y, x) / DELTA_ALPHA

if (math.pi % DELTA_ALPHA != 0):
    raise ValueError("Value of DELTA_ALPHA:", DELTA_ALPHA, "does not divide a circle without remainder.")

segments = [None] * int(M) # the set of segments of points, i.e., s[0] is the set of all points in the first segment

# for all points, assign them to a segment
for i in range(len(test_data)):
    x = test_data[i][0]
    y = test_data[i][1]
    segmentIndex = int(get_segment(x, y)) # I assume, but should check that get_segment always returns an int
    segments[] = test_data[i]

BIN_RANGE = 1 # the length of a bin (in m) # might not be m, unsure of how lidar data is formatted yet

# this function is pretty useless atm, will replace with better one
# Returns true if the point is in Bin j (Note: bins don't need to be defined by the segment they're in. They're defined by distance from origin)
# Each segment will have the same number of bins at the same distances away
def in_bin(x, y, j):
    return (j*BIN_RANGE <= math.sqrt((x**2)+(y**2)) <= (j+1)*BIN_RANGE)





