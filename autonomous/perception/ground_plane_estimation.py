# %%
# - Modules -
import math
import copy

# Plotting Data
from mpl_toolkits.mplot3d import axes3d
import numpy as np
import matplotlib.pyplot as plt

# Line Fitting
import line_extraction

test_data = [[-1, -0.25, 0.6], [-1.5, -0.4, 0.6], [-0.25, 0.35, 0.3], [-0.25, 0.5, 0.4], [0.1, 0.25, 0.4], [0.1, 0.75, 0.41], [0.1, 1.25, 0.42], [0.1, 1.75, 0.43], [0.1, 2.25, 0.44], [0.1, 2.75, 0.45], [-0.5, 1, 0.5], [-0.5, -0.25, 0.5], [-0.5, -1, 0.5,], [1, -0.5, 0.5], [0.5, 0.5, 0.5], [0.5, 1, 0.5], [0.5, 1, 1], [1, 0.5, 0.5], [1, 0.5, 1], [1, 1, 0.5], [1, 1, 1], [0.5, 0.5, -1], [0.5, -1, 0.5], [0.5, -1, -1], [-1, 0.5, 0.5], [-1, 0.5, -1], [-1, -1, 0.5], [-1, -1, -1], [0.5, -1, 1], [-1, 0.5, 1], [-1, 1, 0.5], [-1, 1, 1], [0.5, 1, -1], [1, 0.5, -1], [1, -1, 0.5], [1, -1, 1], [1, 1, -1], [-1, -1, 1], [1, -1, -1], [-1, 1, -1], [0.5, 0.5, 1.1], [0.5, 1.1, 0.5], [0.5, 1.1, 1.1], [1.1, 0.5, 0.5], [1.1, 0.5, 1.1], [1.1, 1.1, 0.5], [1.1, 1.1, 1.1], [0.5, 0.5, -1.1], [0.5, -1.1, 0.5], [0.5, -1.1, -1.1], [-1.1, 0.5, 0.5], [-1.1, 0.5, -1.1], [-1.1, -1.1, 0.5], [-1.1, -1.1, -1.1], [0.5, -1.1, 1.1], [-1.1, 0.5, 1.1], [-1.1, 1.1, 0.5], [-1.1, 1.1, 1.1], [0.5, 1.1, -1.1], [1.1, 0.5, -1.1], [1.1, -1.1, 0.5], [1.1, -1.1, 1.1], [1.1, 1.1, -1.1], [-1.1, -1.1, 1.1], [1.1, -1.1, -1.1], [-1.1, 1.1, -1.1], [0.5, 0.5, 1.5], [0.5, 1.5, 0.5], [0.5, 1.5, 1.5], [1.5, 0.5, 0.5], [1.5, 0.5, 1.5], [1.5, 1.5, 0.5], [1.5, 1.5, 1.5], [0.5, 0.5, -1.5], [0.5, -1.5, 0.5], [0.5, -1.5, -1.5], [-1.5, 0.5, 0.5], [-1.5, 0.5, -1.5], [-1.5, -1.5, 0.5], [-1.5, -1.5, -1.5], [0.5, -1.5, 1.5], [-1.5, 0.5, 1.5], [-1.5, 1.5, 0.5], [-1.5, 1.5, 1.5], [0.5, 1.5, -1.5], [1.5, 0.5, -1.5], [1.5, -1.5, 0.5], [1.5, -1.5, 1.5], [1.5, 1.5, -1.5], [-1.5, -1.5, 1.5], [1.5, -1.5, -1.5], [-1.5, 1.5, -1.5], [0.5, 0.5, 2], [0.5, 2, 0.5], [0.5, 2, 2], [2, 0.5, 0.5], [2, 0.5, 2], [2, 2, 0.5], [2, 2, 2], [0.5, 0.5, -2], [0.5, -2, 0.5], [0.5, -2, -2], [-2, 0.5, 0.5], [-2, 0.5, -2], [-2, -2, 0.5], [-2, -2, -2], [0.5, -2, 2], [-2, 0.5, 2], [-2, 2, 0.5], [-2, 2, 2], [0.5, 2, -2], [2, 0.5, -2], [2, -2, 0.5], [2, -2, 2], [2, 2, -2], [-2, -2, 2], [2, -2, -2], [-2, 2, -2]]
#test_data = [[1, 0, 0.7], [1.25, 0, 0.6], [1.5, 0, 0.5], [2, 0, 0.4], [-1, -0.25, 0.6], [-1.5, -0.4, 0.6], [-0.25, 0.35, 0.3], [-0.25, 0.5, 0.4], [0.1, 0.25, 0.4], [0.1, 0.75, 0.41], [0.1, 1.25, 0.42], [0.1, 1.75, 0.43], [0.1, 2.25, 0.44], [0.1, 2.75, 0.45], [-0.5, 1, 0.5], [-0.5, -0.25, 0.5], [-0.5, -1, 0.5,], [1, -0.5, 0.5], [0.5, 0.5, 0.5], [0.5, 1, 0.5], [0.5, 1, 1], [1, 1, 0.5], [1, 1, 1], [0.5, 0.5, -1], [0.5, -1, 0.5], [0.5, -1, -1], [-1, 0.5, 0.5], [-1, 0.5, -1], [-1, -1, 0.5], [-1, -1, -1], [0.5, -1, 1], [-1, 0.5, 1], [-1, 1, 0.5], [-1, 1, 1], [0.5, 1, -1], [1, -1, 0.5], [1, -1, 1], [1, 1, -1], [-1, -1, 1], [1, -1, -1], [-1, 1, -1], [0.5, 0.5, 1.1], [0.5, 1.1, 0.5], [0.5, 1.1, 1.1], [1.1, 1.1, 0.5], [1.1, 1.1, 1.1], [0.5, 0.5, -1.1], [0.5, -1.1, 0.5], [0.5, -1.1, -1.1], [-1.1, 0.5, 0.5], [-1.1, 0.5, -1.1], [-1.1, -1.1, 0.5], [-1.1, -1.1, -1.1], [0.5, -1.1, 1.1], [-1.1, 0.5, 1.1], [-1.1, 1.1, 0.5], [-1.1, 1.1, 1.1], [0.5, 1.1, -1.1], [1.1, -1.1, 0.5], [1.1, -1.1, 1.1], [1.1, 1.1, -1.1], [-1.1, -1.1, 1.1], [1.1, -1.1, -1.1], [-1.1, 1.1, -1.1], [0.5, 0.5, 1.5], [0.5, 1.5, 0.5], [0.5, 1.5, 1.5], [1.5, 1.5, 0.5], [1.5, 1.5, 1.5], [0.5, 0.5, -1.5], [0.5, -1.5, 0.5], [0.5, -1.5, -1.5], [-1.5, 0.5, 0.5], [-1.5, 0.5, -1.5], [-1.5, -1.5, 0.5], [-1.5, -1.5, -1.5], [0.5, -1.5, 1.5], [-1.5, 0.5, 1.5], [-1.5, 1.5, 0.5], [-1.5, 1.5, 1.5], [0.5, 1.5, -1.5], [1.5, -1.5, 0.5], [1.5, -1.5, 1.5], [1.5, 1.5, -1.5], [-1.5, -1.5, 1.5], [1.5, -1.5, -1.5], [-1.5, 1.5, -1.5], [0.5, 0.5, 2], [0.5, 2, 0.5], [0.5, 2, 2], [2, 2, 0.5], [2, 2, 2], [0.5, 0.5, -2], [0.5, -2, 0.5], [0.5, -2, -2], [-2, 0.5, 0.5], [-2, 0.5, -2], [-2, -2, 0.5], [-2, -2, -2], [0.5, -2, 2], [-2, 0.5, 2], [-2, 2, 0.5], [-2, 2, 2], [0.5, 2, -2], [2, -2, 0.5], [2, -2, 2], [2, 2, -2], [-2, -2, 2], [2, -2, -2], [-2, 2, -2]]
#test_data = [[1, 0, 0.7], [1.25, 0, 0.6], [1.5, 0, 0.5], [2, 0, 0.4], [0, 0.5, 0.7], [0, 1, 0.6], [0, 1.5, 0.5], [0, 2, 0.4]]

# Max value of the norm of x and y (excluding z)
values = []
for i in range(len(test_data)):
    values.append(math.sqrt(test_data[i][0] ** 2 + test_data[i][1] ** 2))

FIGURES_DIR = "./autonomous/perception/figures/"

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
    visualise_data(segments, segments_bins, segments_bins_2D, segments_bins_prototype, extracted_lines)
    return extracted_lines

def visualise_data(segments, segments_bins, segments_bins_2D, segments_bins_prototype, extracted_lines):
    color_codes = ['b', 'g', 'r', 'grey', 'm', 'orange']
    cmaps = ["Blues", "Greens", "Reds", "Greys", "Purples", "Oranges"]
    angle_points = 25

    plot_segments(segments, color_codes, angle_points)
    plot_segments_bins(segments_bins, color_codes, angle_points)
    plot_segments_bins_2D(segments_bins_2D, color_codes, angle_points)
    plot_segments_bins_2D_3D(segments_bins_2D, color_codes, angle_points, cmaps)
    plot_segments_bins_prototype_3D(segments_bins_prototype, color_codes, angle_points, cmaps)
    plot_extracted_lines_3D(segments_bins_prototype, color_codes, angle_points, extracted_lines)
    plot_segments_fitted(segments_bins_prototype, extracted_lines, color_codes)

    plt.show()

# Plotting test_data.

xdata = [coords[0] for coords in test_data]
ydata = [coords[1] for coords in test_data]
zdata = [coords[2] for coords in test_data]

def init_plot_2D(title, xlabel, ylabel):
    fig = plt.figure()
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)

def init_plot_3D(title, xlabel, ylabel, zlabel, azim, elev):
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.view_init(azim=azim, elev=elev)
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_zlabel(zlabel)
    return ax

def plot_data_2D(points):
    x = [coords[0] for coords in points]
    y = [coords[1] for coords in points]
    
    init_plot_2D("Point Cloud (2D)", "x", "y")
    plt.plot(x, y, '.', color='green')
    plt.plot(0, 0, 'o', color='black')
    angles = np.linspace(0, 2*math.pi, 100)
    plt.plot(LIDAR_RANGE*np.cos(angles), LIDAR_RANGE*np.sin(angles), color='black')
    plt.savefig(FIGURES_DIR + "Point_Cloud_2D-1")

def plot_data_3D(points):
    x = [coords[0] for coords in points]
    y = [coords[1] for coords in points]
    z = [coords[2] for coords in points]

    ax = init_plot_3D("Point Cloud", "x", "y", "Height", 45, 45)
    ax.scatter3D(x, y, z, c=z, cmap='Greens');
    
    angles = np.linspace(0, 2*math.pi, 100)
    ax.plot3D(LIDAR_RANGE * np.cos(angles), LIDAR_RANGE * np.sin(angles), color='black')
    plt.savefig(FIGURES_DIR + "Point_Cloud_3D-2")

def plot_segments(segments, color_codes, angle_points):
    init_plot_2D("Points Assigned to Segments", "x", "y")
    for i in range(len(segments)):
        color = color_codes[i % len(color_codes)]
        x = [coords[0] for coords in segments[i]]
        y = [coords[1] for coords in segments[i]]
        plt.plot(x, y, '.', color=color)
        xpoints = [0, LIDAR_RANGE * math.cos(i * DELTA_ALPHA)]
        ypoints = [0, LIDAR_RANGE * math.sin(i * DELTA_ALPHA)]
        plt.plot(xpoints, ypoints, color=color)
        angles = np.linspace(i * DELTA_ALPHA, (i + 1) * DELTA_ALPHA, angle_points)
        plt.plot(LIDAR_RANGE*np.cos(angles), LIDAR_RANGE*np.sin(angles), color=color)
    plt.savefig(FIGURES_DIR + "Segments-3")

def plot_segments_bins(segments_bins, color_codes, angle_points):
    init_plot_2D("Points Assigned to Bins within Segments", "x", "y")

    for i in range(len(segments_bins)):
        color1 = color_codes[i % len(color_codes)]
        angles = np.linspace(i * DELTA_ALPHA, (i + 1) * DELTA_ALPHA, angle_points)
        for j in range(len(segments_bins[i])):
            color2 = color_codes[j % len(color_codes)]
            x = [coords[0] for coords in segments_bins[i][j]]
            y = [coords[1] for coords in segments_bins[i][j]]
            plt.plot(x, y, '.', color=color2)
            plt.plot((j * BIN_SIZE) * np.cos(angles), (j * BIN_SIZE) * np.sin(angles), color=color1)
        xpoints = [0, LIDAR_RANGE * math.cos(i * DELTA_ALPHA)]
        ypoints = [0, LIDAR_RANGE * math.sin(i * DELTA_ALPHA)]
        plt.plot(xpoints, ypoints, color=color1)
        plt.plot(LIDAR_RANGE*np.cos(angles), LIDAR_RANGE*np.sin(angles), color=color1)
    plt.savefig(FIGURES_DIR + "Bins_Segments-4")

def plot_segments_bins_2D(segments_bins_2D, color_codes, angle_points):
    init_plot_2D("2D Approximation of Point Cloud", "x", "y")

    for i in range(len(segments_bins_2D)):
        color1 = color_codes[i % len(color_codes)]
        angles = np.linspace(i * DELTA_ALPHA, (i + 1) * DELTA_ALPHA, angle_points)
        for j in range(len(segments_bins_2D[i])):
            color2 = color_codes[j % len(color_codes)]
            norm = [coords[0] for coords in segments_bins_2D[i][j]]
            new_x = []
            new_y = []
            for k in range(len(norm)):
                new_x.append(norm[k] * math.cos((i + 0.5) * DELTA_ALPHA))
                new_y.append(norm[k] * math.sin((i + 0.5) * DELTA_ALPHA))
            plt.plot(new_x, new_y, '.', color=color2)
            plt.plot((j * BIN_SIZE) * np.cos(angles), (j * BIN_SIZE) * np.sin(angles), color=color1)
        xpoints1 = [0, LIDAR_RANGE * math.cos(i * DELTA_ALPHA)]
        ypoints1 = [0, LIDAR_RANGE * math.sin(i * DELTA_ALPHA)]
        plt.plot(xpoints1, ypoints1, color=color1)
        xpoints2 = [0, LIDAR_RANGE * math.cos((i + 0.5) * DELTA_ALPHA)]
        ypoints2 = [0, LIDAR_RANGE * math.sin((i + 0.5) * DELTA_ALPHA)]
        plt.plot(xpoints2, ypoints2, color='black', alpha=0.5, linestyle='--')
        plt.plot(LIDAR_RANGE*np.cos(angles), LIDAR_RANGE*np.sin(angles), color=color1)
    plt.savefig(FIGURES_DIR + "2D_Approx_Point_Cloud_2D-5")

def plot_segments_bins_2D_3D(segments_bins_2D, color_codes, angle_points, cmaps):
    ax = init_plot_3D("2D Approximation of Point Cloud", "x", "y", "Height", 45, 45)

    for i in range(len(segments_bins_2D)):
        color1 = color_codes[i % len(color_codes)]
        cmap1 = cmaps[i % len(cmaps)]
        angles = np.linspace(i * DELTA_ALPHA, (i + 1) * DELTA_ALPHA, angle_points)
        for j in range(len(segments_bins_2D[i])):
            norm = [coords[0] for coords in segments_bins_2D[i][j]]
            z = [coords[1] for coords in segments_bins_2D[i][j]]
            new_x = []
            new_y = []
            for k in range(len(norm)):
                new_x.append(norm[k] * math.cos((i + 0.5) * DELTA_ALPHA))
                new_y.append(norm[k] * math.sin((i + 0.5) * DELTA_ALPHA))
            ax.scatter3D(new_x, new_y, z, c=z, cmap=cmap1);
            ax.plot3D((j * BIN_SIZE) * np.cos(angles), (j * BIN_SIZE) * np.sin(angles), color=color1)
    plt.savefig(FIGURES_DIR + "2D_Approx_Point_Cloud_3D-6")

def plot_segments_bins_prototype_3D(segments_bins_prototype, color_codes, angle_points, cmaps):
    ax = init_plot_3D("Prototype points", "x", "y", "Height", 45, 45)

    for i in range(len(segments_bins_prototype)):
        color1 = color_codes[i % len(color_codes)]
        angles = np.linspace(i * DELTA_ALPHA, (i + 1) * DELTA_ALPHA, angle_points)
        for j in range(len(segments_bins_prototype[i])):
            print(segments_bins_prototype[i])
            if len(segments_bins_prototype[i][j]) > 0:
                norm = segments_bins_prototype[i][j][0]
                z = segments_bins_prototype[i][j][1]
                new_x = norm * math.cos((i + 0.5) * DELTA_ALPHA)
                new_y = norm * math.sin((i + 0.5) * DELTA_ALPHA)
                ax.scatter3D(new_x, new_y, z, c=z, cmap='viridis');
            ax.plot3D((j * BIN_SIZE) * np.cos(angles), (j * BIN_SIZE) * np.sin(angles), color=color1)
    plt.savefig(FIGURES_DIR + "Prototype_Points-7")

def plot_extracted_lines_3D(segments_bins_prototype, color_codes, angle_points, extracted_lines):
    ax = init_plot_3D("Ground Plane Estimation (Prototype Points)", "x", "y", "Height", 45, 45)

    for i in range(len(segments_bins_prototype)):
        color1 = color_codes[i % len(color_codes)]
        angles = np.linspace(i * DELTA_ALPHA, (i + 1) * DELTA_ALPHA, angle_points)
        for j in range(len(segments_bins_prototype[i])):
            print(segments_bins_prototype[i])
            if len(segments_bins_prototype[i][j]) > 0:
                norm = segments_bins_prototype[i][j][0]
                z = segments_bins_prototype[i][j][1]
                new_x = norm * math.cos((i + 0.5) * DELTA_ALPHA)
                new_y = norm * math.sin((i + 0.5) * DELTA_ALPHA)
                ax.scatter3D(new_x, new_y, z, c=z, cmap='viridis');
            ax.plot3D((j * BIN_SIZE) * np.cos(angles), (j * BIN_SIZE) * np.sin(angles), color=color1)

    print("hey", extracted_lines)

    for i in range(len(extracted_lines)):
        for j in range(len(extracted_lines[i])):
            r = np.linspace(0, LIDAR_RANGE, 50)
            z = extracted_lines[i][j][0] * r + extracted_lines[i][j][1]
            x = r * math.cos((i + 0.5) * DELTA_ALPHA)
            y = r * math.sin((i + 0.5) * DELTA_ALPHA)
            ax.plot3D(x, y, z, color='black')

    plt.savefig(FIGURES_DIR + "Ground_Plane_Estimation-8")

def plot_segments_fitted(segments_bins_prototype, extracted_lines, color_codes):
    print("SP", segments_bins_prototype)
    print("--- Prototype Points ---")
    for i in range(len(segments_bins_prototype)):
        color1 = color_codes[i % len(color_codes)]
        print("Segment:", i)
        init_plot_2D("Segment " + str(i + 1) + " | Degrees: " + str(round((i * DELTA_ALPHA) * 180/math.pi, 2)) + " to " + str(round((i + 1) * DELTA_ALPHA * 180/math.pi, 2)), "Distance from origin", "Height")
        x = []
        y = []
        for j in range(len(segments_bins_prototype[i])):
            if len(segments_bins_prototype[i][j]) > 0:
                x.append(segments_bins_prototype[i][j][0])
                y.append(segments_bins_prototype[i][j][1])
                print("     Bin:", j, "Point:", segments_bins_prototype[i][j][0], segments_bins_prototype[i][j][1])
                for j in range(len(extracted_lines[i])):
                    x_gp = np.linspace(0, LIDAR_RANGE, 50)
                    y_gp = extracted_lines[i][j][0] * x_gp + extracted_lines[i][j][1]
                    plt.plot(x_gp, y_gp, '--', color=color1)
        plt.plot(x, y, 'o', color='black')
        plt.savefig(FIGURES_DIR + "Segment_" + str(i+1) + "_Fitted_Line")
        
# Main
plot_data_2D(test_data)
plot_data_3D(test_data)
ground_plane = get_ground_plane(test_data)
print(ground_plane)

# p1 = [1, 1.5]
# p2 = [2, 2]
# p3 = [3, 2.5]
# p4 = [4, 3]
# p5 = [5, 3.5]
# test_line = [p1, p2, p3, p4, p5]
# extracted_lines = line_extraction.extract_lines([test_line], 1, 5)
# print("Line:", extracted_lines)

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