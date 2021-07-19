# %%
# Modules   
import math
import copy

from matplotlib.colors import LightSource
import line_extraction

test_data = [[-1, -0.25, 0.6], [-1.5, -0.4, 0.6], [-0.25, 0.35, 0.3], [-0.25, 0.5, 0.4], [0.1, 0.25, 0.4], [0.1, 0.75, 0.41], [0.1, 1.25, 0.42], [0.1, 1.75, 0.43], [0.1, 2.25, 0.44], [0.1, 2.75, 0.45], [-0.5, 1, 0.5], [-0.5, -0.25, 0.5], [-0.5, -1, 0.5,], [1, -0.5, 0.5], [0.5, 0.5, 0.5], [0.5, 1, 0.5], [0.5, 1, 1], [1, 0.5, 0.5], [1, 0.5, 1], [1, 1, 0.5], [1, 1, 1], [0.5, 0.5, -1], [0.5, -1, 0.5], [0.5, -1, -1], [-1, 0.5, 0.5], [-1, 0.5, -1], [-1, -1, 0.5], [-1, -1, -1], [0.5, -1, 1], [-1, 0.5, 1], [-1, 1, 0.5], [-1, 1, 1], [0.5, 1, -1], [1, 0.5, -1], [1, -1, 0.5], [1, -1, 1], [1, 1, -1], [-1, -1, 1], [1, -1, -1], [-1, 1, -1], [0.5, 0.5, 1.1], [0.5, 1.1, 0.5], [0.5, 1.1, 1.1], [1.1, 0.5, 0.5], [1.1, 0.5, 1.1], [1.1, 1.1, 0.5], [1.1, 1.1, 1.1], [0.5, 0.5, -1.1], [0.5, -1.1, 0.5], [0.5, -1.1, -1.1], [-1.1, 0.5, 0.5], [-1.1, 0.5, -1.1], [-1.1, -1.1, 0.5], [-1.1, -1.1, -1.1], [0.5, -1.1, 1.1], [-1.1, 0.5, 1.1], [-1.1, 1.1, 0.5], [-1.1, 1.1, 1.1], [0.5, 1.1, -1.1], [1.1, 0.5, -1.1], [1.1, -1.1, 0.5], [1.1, -1.1, 1.1], [1.1, 1.1, -1.1], [-1.1, -1.1, 1.1], [1.1, -1.1, -1.1], [-1.1, 1.1, -1.1], [0.5, 0.5, 1.5], [0.5, 1.5, 0.5], [0.5, 1.5, 1.5], [1.5, 0.5, 0.5], [1.5, 0.5, 1.5], [1.5, 1.5, 0.5], [1.5, 1.5, 1.5], [0.5, 0.5, -1.5], [0.5, -1.5, 0.5], [0.5, -1.5, -1.5], [-1.5, 0.5, 0.5], [-1.5, 0.5, -1.5], [-1.5, -1.5, 0.5], [-1.5, -1.5, -1.5], [0.5, -1.5, 1.5], [-1.5, 0.5, 1.5], [-1.5, 1.5, 0.5], [-1.5, 1.5, 1.5], [0.5, 1.5, -1.5], [1.5, 0.5, -1.5], [1.5, -1.5, 0.5], [1.5, -1.5, 1.5], [1.5, 1.5, -1.5], [-1.5, -1.5, 1.5], [1.5, -1.5, -1.5], [-1.5, 1.5, -1.5], [0.5, 0.5, 2], [0.5, 2, 0.5], [0.5, 2, 2], [2, 0.5, 0.5], [2, 0.5, 2], [2, 2, 0.5], [2, 2, 2], [0.5, 0.5, -2], [0.5, -2, 0.5], [0.5, -2, -2], [-2, 0.5, 0.5], [-2, 0.5, -2], [-2, -2, 0.5], [-2, -2, -2], [0.5, -2, 2], [-2, 0.5, 2], [-2, 2, 0.5], [-2, 2, 2], [0.5, 2, -2], [2, 0.5, -2], [2, -2, 0.5], [2, -2, 2], [2, 2, -2], [-2, -2, 2], [2, -2, -2], [-2, 2, -2]]
#test_data = [[1, 0, 0.7], [1.25, 0, 0.6], [1.5, 0, 0.5], [2, 0, 0.4], [-1, -0.25, 0.6], [-1.5, -0.4, 0.6], [-0.25, 0.35, 0.3], [-0.25, 0.5, 0.4], [0.1, 0.25, 0.4], [0.1, 0.75, 0.41], [0.1, 1.25, 0.42], [0.1, 1.75, 0.43], [0.1, 2.25, 0.44], [0.1, 2.75, 0.45], [-0.5, 1, 0.5], [-0.5, -0.25, 0.5], [-0.5, -1, 0.5,], [1, -0.5, 0.5], [0.5, 0.5, 0.5], [0.5, 1, 0.5], [0.5, 1, 1], [1, 1, 0.5], [1, 1, 1], [0.5, 0.5, -1], [0.5, -1, 0.5], [0.5, -1, -1], [-1, 0.5, 0.5], [-1, 0.5, -1], [-1, -1, 0.5], [-1, -1, -1], [0.5, -1, 1], [-1, 0.5, 1], [-1, 1, 0.5], [-1, 1, 1], [0.5, 1, -1], [1, -1, 0.5], [1, -1, 1], [1, 1, -1], [-1, -1, 1], [1, -1, -1], [-1, 1, -1], [0.5, 0.5, 1.1], [0.5, 1.1, 0.5], [0.5, 1.1, 1.1], [1.1, 1.1, 0.5], [1.1, 1.1, 1.1], [0.5, 0.5, -1.1], [0.5, -1.1, 0.5], [0.5, -1.1, -1.1], [-1.1, 0.5, 0.5], [-1.1, 0.5, -1.1], [-1.1, -1.1, 0.5], [-1.1, -1.1, -1.1], [0.5, -1.1, 1.1], [-1.1, 0.5, 1.1], [-1.1, 1.1, 0.5], [-1.1, 1.1, 1.1], [0.5, 1.1, -1.1], [1.1, -1.1, 0.5], [1.1, -1.1, 1.1], [1.1, 1.1, -1.1], [-1.1, -1.1, 1.1], [1.1, -1.1, -1.1], [-1.1, 1.1, -1.1], [0.5, 0.5, 1.5], [0.5, 1.5, 0.5], [0.5, 1.5, 1.5], [1.5, 1.5, 0.5], [1.5, 1.5, 1.5], [0.5, 0.5, -1.5], [0.5, -1.5, 0.5], [0.5, -1.5, -1.5], [-1.5, 0.5, 0.5], [-1.5, 0.5, -1.5], [-1.5, -1.5, 0.5], [-1.5, -1.5, -1.5], [0.5, -1.5, 1.5], [-1.5, 0.5, 1.5], [-1.5, 1.5, 0.5], [-1.5, 1.5, 1.5], [0.5, 1.5, -1.5], [1.5, -1.5, 0.5], [1.5, -1.5, 1.5], [1.5, 1.5, -1.5], [-1.5, -1.5, 1.5], [1.5, -1.5, -1.5], [-1.5, 1.5, -1.5], [0.5, 0.5, 2], [0.5, 2, 0.5], [0.5, 2, 2], [2, 2, 0.5], [2, 2, 2], [0.5, 0.5, -2], [0.5, -2, 0.5], [0.5, -2, -2], [-2, 0.5, 0.5], [-2, 0.5, -2], [-2, -2, 0.5], [-2, -2, -2], [0.5, -2, 2], [-2, 0.5, 2], [-2, 2, 0.5], [-2, 2, 2], [0.5, 2, -2], [2, -2, 0.5], [2, -2, 2], [2, 2, -2], [-2, -2, 2], [2, -2, -2], [-2, 2, -2]]
#test_data = [[1, 0, 0.7], [1.25, 0, 0.6], [1.5, 0, 0.5], [2, 0, 0.4], [0, 0.5, 0.7], [0, 1, 0.6], [0, 1.5, 0.5], [0, 2, 0.4]]

# Max value of the norm of x and y (excluding z)
values = []
for i in range(len(test_data)):
    values.append(math.sqrt(test_data[i][0] ** 2 + test_data[i][1] ** 2))

# Constants
LIDAR_RANGE = 400 # Max range of the LIDAR # 400
LIDAR_RANGE = math.ceil(max(values))
print("Max x-y norm:", LIDAR_RANGE)
DELTA_ALPHA = 2*math.pi / 8 # Angle of each segment # 45 deg
NUM_SEGMENTS = math.ceil(2*math.pi / DELTA_ALPHA) # Number of segments # 8
BIN_SIZE = 0.5 # The length of a bin (in metres) # 1
NUM_BINS = math.ceil(LIDAR_RANGE / BIN_SIZE) # A derived constant

if (math.pi % DELTA_ALPHA != 0):
    raise ValueError("Value of DELTA_ALPHA:", DELTA_ALPHA, "does not divide a circle into an integer-number of segments.")

# Returns the index to a segment that a point maps to
def get_segment(x, y):
    return math.floor(math.atan2(y, x) / DELTA_ALPHA)

# Creates the set of segments for points
def init_segments():
    segments = [] 
    for i in range(NUM_SEGMENTS):
        segments.append([])
    return segments
    
# Assign each point to a segment 
def points_to_segment(points):
    segments = init_segments()
    for i in range(len(points)):
        x = points[i][0]
        y = points[i][1]
        s_index = get_segment(x, y)
        segments[s_index].append(points[i])
    return segments

# Creates a set of segments with bins
def init_bins():
    segments_bins = []
    for i in range(NUM_SEGMENTS):
        segments_bins.append([])
        for j in range(NUM_BINS):
            segments_bins[i].append([])
    print("Number of bins:", len(segments_bins[0]))
    return segments_bins

# Returns true if the point (x, y) is in bin j
def in_bin(x, y, j):
    return (j*BIN_SIZE <= math.sqrt((x**2)+(y**2)) <= (j+1)*BIN_SIZE)

# Returns the index to a bin that a point (x, y) maps to 
def get_bin(x, y):
    norm = math.sqrt((x**2)+(y**2))
    bin_index = math.floor(norm / BIN_SIZE) -1 # -1 since index of 0 represents the first bin
    if norm % BIN_SIZE != 0:
        bin_index += 1
    if bin_index > NUM_BINS:
        bin_index = LIDAR_RANGE
        print("Point exceeds expected max range of LIDAR. bin_index:", bin_index)
    return bin_index

# Assign each point to a bin
def points_to_bins(segments):
    segments_bins = init_bins()
    for i in range(NUM_SEGMENTS):
        for j in range(len(segments[i])):
            x = segments[i][j][0]
            y = segments[i][j][1]
            bin_index = get_bin(x, y)
            segments_bins[i][bin_index].append(segments[i][j])
    return segments_bins

# Creates a 2D approximation of the 3D points
def approximate_2D(segments_bins):
    segments_bins_2D = copy.deepcopy(segments_bins) 
    for i in range(NUM_SEGMENTS):
        for j in range(NUM_BINS):
            for k in range(len(segments_bins[i][j])):
                x = segments_bins[i][j][k][0]
                y = segments_bins[i][j][k][1]
                z = segments_bins[i][j][k][2]
                point_prime = [math.sqrt(x**2 + y**2), z]
                segments_bins_2D[i][j][k] = point_prime
            # This should order the points by range for each bin:
            segments_bins_2D[i][j] = sorted(segments_bins_2D[i][j], key=lambda lam: lam[0])
            segments_bins_2D[i][j].reverse()
    return segments_bins_2D

# Reduces all points in a bin to a single prototype point
def prototype_points(segments_bins_2D):
    segments_bins_prototype = [] 
    for i in range(NUM_SEGMENTS):
        segments_bins_prototype.append([])
        for j in range(NUM_BINS):
            segments_bins_prototype[i].append([])
            if len(segments_bins_2D[i][j]) > 0:
                segments_bins_prototype[i][j] = segments_bins_2D[i][j][0]
    return segments_bins_prototype

def get_ground_plane(points):
    segments = points_to_segment(points)
    segments_bins = points_to_bins(segments)
    segments_bins_2D = approximate_2D(segments_bins)
    segments_bins_prototype = prototype_points(segments_bins_2D)
    extracted_lines = line_extraction.extract_lines(segments_bins_prototype, NUM_SEGMENTS, NUM_BINS)
    return extracted_lines

# Plotting test_data.
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import axes3d
import numpy as np
import matplotlib.pyplot as plt

xdata = [coords[0] for coords in test_data]
ydata = [coords[1] for coords in test_data]
zdata = [coords[2] for coords in test_data]

# Single plot
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.view_init(azim=45, elev=45)

ax.scatter3D(xdata, ydata, zdata, c=zdata, cmap='Greens');

# Rotating plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter3D(xdata, ydata, zdata, c=zdata, cmap='Greens')

for angle in range(0, 360):
    ax.view_init(30, angle)
    plt.draw()
    plt.pause(0.0005)

def abline(slope, intercept):
    """Plot a line from slope and intercept"""
    axes = plt.gca()
    x_vals = np.array(axes.get_xlim())
    y_vals = intercept + slope * x_vals
    plt.plot(x_vals, y_vals, '--')  

fig = plt.figure()
ax = plt.axes(projection='3d')
#ax.view_init(azim=0, elev=90)
ax.scatter(xdata, ydata, zdata, c='green')
ax.plot3D(gp_x, gp_y, gp_z, c='pink')

angles = np.linspace(0, 2*math.pi, 100)
ax.plot3D(DELTA_ALPHA*np.cos(angles), DELTA_ALPHA*np.sin(angles), -DELTA_ALPHA*np.ones(angles.shape), color='red')
ax.plot3D(DELTA_ALPHA*np.cos(angles), DELTA_ALPHA*np.sin(angles), np.zeros(angles.shape), color='red')
ax.plot3D(DELTA_ALPHA*np.cos(angles), DELTA_ALPHA*np.sin(angles), DELTA_ALPHA*np.ones(angles.shape), color='red')

for i in range(NUM_SEGMENTS):    
    xpoints = [0, DELTA_ALPHA * math.cos(i*angle)]
    ypoints = [0, DELTA_ALPHA * math.sin(i*angle)]
    ax.plot3D(xpoints, ypoints, [-DELTA_ALPHA, -DELTA_ALPHA], color='black')
    ax.plot3D(xpoints, ypoints, [0, 0], color='black')
    ax.plot3D(xpoints, ypoints, [DELTA_ALPHA, DELTA_ALPHA], color='black')

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_xlim(-DELTA_ALPHA,DELTA_ALPHA)
ax.set_ylim(-DELTA_ALPHA,DELTA_ALPHA)
ax.set_zlim(-DELTA_ALPHA,DELTA_ALPHA)


# %%

# Notes
#   - Constant M: M must be an int, the value error below it
#     checks for this (I think).
#   - get_segment(): Returns negatives but Python lists handle 
#     this and wraps around.
#   - Var segments: segments[0] is the set of all points in the 
#     first segment.
#   - get_segment(): I assume, but should check that this always 
#     returns an int
#   - Constant BIN_SIZE: Might not be a value in metres, depends 
#     on the encoding of LIDAR data. 
#   - in_bin(): Note: bins don't need to be defined by the segment 
#     they're in. They're defined by the distance from the origin.
#     Each segment will have the same number of bins spaced at the 
#     same distances away from origin. 
#     This function is currently not needed. 
#   - Constant NUM_BINS: # This NEEDS to  be an int. Perhaps I could 
#     round down or have the last bin be smaller?
#   - approximate_2D(): This 3D to 2D conversion is a propety of 
#     the bins. The greater number of bins, the more accurate the 
#     approximation. This conversion is done by getting the distance 
#     x and y are from the origin and a prime point is simply a 2 
#     dimensional vector of this norm and the z height. This function 
#     also orders the points in each bin in ascending range 
#     (distance from origin)
#   - prototype_points(): point-to-bin mapping is not one-to-one, 
#     and P_(b_j^s)^' may contain more than one point, or no points 
#     at all. A prototype point will be the point within a bin with
#     the lowest z value (height) as this point is most likely to lie
#     on the plane. 