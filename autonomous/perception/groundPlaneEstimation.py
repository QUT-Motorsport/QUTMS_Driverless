import math
import copy

# many points [x, y, z]
test_data = [[0, 0, 1], [0, 1, 0], [0, 1, 1], [1, 0, 0], [1, 0, 1], [1, 1, 0], [1, 1, 1], [0, 0, -1], [0, -1, 0], [0, -1, -1], [-1, 0, 0], [-1, 0, -1], [-1, -1, 0], [-1, -1, -1], [0, -1, 1], [-1, 0, 1], [-1, 1, 0], [-1, 1, 1], [0, 1, -1], [1, 0, -1], [1, -1, 0], [1, -1, 1], [1, 1, -1], [-1, -1, 1], [1, -1, -1], [-1, 1, -1], [0, 0, 1.1], [0, 1.1, 0], [0, 1.1, 1.1], [1.1, 0, 0], [1.1, 0, 1.1], [1.1, 1.1, 0], [1.1, 1.1, 1.1], [0, 0, -1.1], [0, -1.1, 0], [0, -1.1, -1.1], [-1.1, 0, 0], [-1.1, 0, -1.1], [-1.1, -1.1, 0], [-1.1, -1.1, -1.1], [0, -1.1, 1.1], [-1.1, 0, 1.1], [-1.1, 1.1, 0], [-1.1, 1.1, 1.1], [0, 1.1, -1.1], [1.1, 0, -1.1], [1.1, -1.1, 0], [1.1, -1.1, 1.1], [1.1, 1.1, -1.1], [-1.1, -1.1, 1.1], [1.1, -1.1, -1.1], [-1.1, 1.1, -1.1], [0, 0, 1.5], [0, 1.5, 0], [0, 1.5, 1.5], [1.5, 0, 0], [1.5, 0, 1.5], [1.5, 1.5, 0], [1.5, 1.5, 1.5], [0, 0, -1.5], [0, -1.5, 0], [0, -1.5, -1.5], [-1.5, 0, 0], [-1.5, 0, -1.5], [-1.5, -1.5, 0], [-1.5, -1.5, -1.5], [0, -1.5, 1.5], [-1.5, 0, 1.5], [-1.5, 1.5, 0], [-1.5, 1.5, 1.5], [0, 1.5, -1.5], [1.5, 0, -1.5], [1.5, -1.5, 0], [1.5, -1.5, 1.5], [1.5, 1.5, -1.5], [-1.5, -1.5, 1.5], [1.5, -1.5, -1.5], [-1.5, 1.5, -1.5], [0, 0, 2], [0, 2, 0], [0, 2, 2], [2, 0, 0], [2, 0, 2], [2, 2, 0], [2, 2, 2], [0, 0, -2], [0, -2, 0], [0, -2, -2], [-2, 0, 0], [-2, 0, -2], [-2, -2, 0], [-2, -2, -2], [0, -2, 2], [-2, 0, 2], [-2, 2, 0], [-2, 2, 2], [0, 2, -2], [2, 0, -2], [2, -2, 0], [2, -2, 2], [2, 2, -2], [-2, -2, 2], [2, -2, -2], [-2, 2, -2]]

LIDAR_RANGE = 10 # the max range of the LiDAR 

DELTA_ALPHA = math.pi / 4 # the angle every segment covers (45deg)
M = int(2*math.pi / DELTA_ALPHA) # number of segments (8) # let's hope M is an int, the value error below checks it i think, im tired.

# Returns the index to a segment that a point maps to
# Returns negatives but Python lists handle this and wrap around
def get_segment(x, y):
    return math.atan2(y, x) / DELTA_ALPHA

if (math.pi % DELTA_ALPHA != 0):
    raise ValueError("Value of DELTA_ALPHA:", DELTA_ALPHA, "does not divide a circle without remainder.")

# the set of segments of points, i.e., s[0] is the set of all points in the first segment
segments = [] 
for i in range(M):
    segments.append([])

# for all points, assign them to a segment
for i in range(len(test_data)):
    x = test_data[i][0]
    y = test_data[i][1]
    s_index = int(get_segment(x, y)) # I assume, but should check that get_segment always returns an int
    segments[s_index].append(test_data[i])

#print(segments)

# Now need to sort the points in each segment into their respective bins ...

BIN_SIZE = 1 # the length of a bin (in m) # might not be m, unsure of how lidar data is formatted yet

# Returns true if the point is in Bin j (Note: bins don't need to be defined by the segment they're in. They're defined by distance from origin)
# Each segment will have the same number of bins at the same distances away
# function currently not needed
def in_bin(x, y, j):
    return (j*BIN_SIZE <= math.sqrt((x**2)+(y**2)) <= (j+1)*BIN_SIZE)

# Returns the index to a bin that a point maps to 
def get_bin(x, y):
    norm = math.sqrt((x**2)+(y**2))
    bin_index = math.floor(norm / BIN_SIZE) -1 # -1 since index of 0 represents the first bin
    if norm % BIN_SIZE != 0:
        bin_index += 1
    return bin_index

num_bins = int(LIDAR_RANGE / BIN_SIZE) # NEEDS TO BE AN INT, perhaps round down or have the last bin be smaller?

segments_bins = []
for i in range(M):
    segments_bins.append([])
    for j in range(num_bins):
        segments_bins[i].append([])

# assign each point to a bin
for i in range(M):
    for j in range(len(segments[i])):
        x = segments[i][j][0]
        y = segments[i][j][1]
        bin_index = get_bin(x, y)
        segments_bins[i][bin_index].append(segments[i][j])

print(segments_bins)
print("\n\n\n")

# Next is to convert these 3D points in the bins to 2D points
# This is done by getting the distance x and y are from the origin and a prime point 
# is simply a 2 dimensional vector of this norm and the z height
# We are reducing the 'resolution' of our data down to the number of bins we have
# This function also orders the points in each bin in ascending range (distance from origin)
segments_bins_prime = copy.deepcopy(segments_bins) # bruh referencing always trips me up
for i in range(M):
    for j in range(num_bins):
        for k in range(len(segments_bins[i][j])):
            x = segments_bins[i][j][k][0]
            y = segments_bins[i][j][k][1]
            z = segments_bins[i][j][k][2]
            point_prime = [math.sqrt(x**2 + y**2), z]
            segments_bins_prime[i][j][k] = point_prime
        segments_bins_prime[i][j] = sorted(segments_bins_prime[i][j], key=lambda lam: lam[0]) # This should order by range for each bin

print(segments_bins_prime)






# Todo
# - If a datapoint is at origin, [0, 0, 0] it might break get_segment. Although a data point of [0, 0, 0] shouldn't be possible. 