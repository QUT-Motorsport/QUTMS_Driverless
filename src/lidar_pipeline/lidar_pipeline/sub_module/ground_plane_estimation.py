# Modules
import math
import copy
import time

# Plotting Data
import numpy as np
import matplotlib.pyplot as plt

# Line Fitting
from .line_extraction import *


FIGURES_DIR = "./driverless_ws/src/lidar_pipeline/lidar_pipeline/sub_module/figures/"

# Constants
LIDAR_RANGE = 150 # Max range of the LIDAR # 100 # in metres
DELTA_ALPHA = 2*math.pi / 8 # Angle of each segment # 45 deg
NUM_SEGMENTS = math.ceil(2*math.pi / DELTA_ALPHA) # Number of segments # 8
BIN_SIZE = 10 # The length of a bin (in metres) # 1
NUM_BINS = math.ceil(LIDAR_RANGE / BIN_SIZE) # A derived constant
T_D_GROUND = 2 # Maximum distance between point and line to be considered part of ground plane. # 2
T_D_MAX = 100 # Maximum distance a point can be from origin to even be considered for ground plane labelling. Otherwise it's automatically labelled as non-ground. 


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

# Not sure if this is working correctly
# Should compare 3D distance between points and start of line point
def dist_points_3D(x1, x2, m, b):
    x2_z = m*x2[0] + b
    distance = math.sqrt((x2[0] - x1[0])**2 + (x2[1] - x1[1])**2 + (x2_z - x1[2])**2)
    return distance

# distance from point to line in 3D space
def point_line_dist_3D():
    pass

# might need to investigate line extraction distance from point to line, making large values.

# Can there be multiple lines PER segment?? I still don't know and this will
# directly affect how this function is written. 
# But assuming that there is multiple lines ...
def label_points(segments, ground_lines):
    labelled_points = copy.deepcopy(segments)

    # Assuming multiple lines in one segment
    # Identifying closest line for each point
    for i in range(NUM_SEGMENTS):
        num_points = len(segments[i])
        for j in range(num_points):
            point = segments[i][j]
            is_ground = False
            closest_line = None
            num_lines = len(ground_lines[i])
            if num_lines > 0:
                line = ground_lines[i][0]
                closest_line = line
                closest_dist = dist_points_3D(point, line[2], line[0], line[1])
                for k in range(1, num_lines):
                    line = ground_lines[i][k]
                    dist_to_line = dist_points_3D(point, line[2], line[0], line[1])
                    if (dist_to_line < closest_dist):
                        closest_line = line
                        closest_dist = dist_to_line
                hacky_point = [math.sqrt(point[0]**2 + point[1]**2), point[2]] # fix this
                point_to_line_dist = dist_point_line(hacky_point, closest_line[0], closest_line[1])
                if (closest_dist < T_D_MAX and point_to_line_dist < T_D_GROUND):
                    is_ground = True
            labelled_points[i][j].append(is_ground)
    
    return labelled_points

def get_ground_plane(points):
    segments = points_to_segment(points)
    segments_bins = points_to_bins(segments)
    segments_bins_2D = approximate_2D(segments_bins)
    segments_bins_prototype = prototype_points(segments_bins_2D)
    ground_lines = extract_lines(segments_bins_prototype, NUM_SEGMENTS, NUM_BINS)
    print("\n\n\n")
    visualise_data(segments, segments_bins, segments_bins_2D, segments_bins_prototype, ground_lines)
    return ground_lines

def visualise_data(segments, segments_bins, segments_bins_2D, segments_bins_prototype, ground_lines):
    color_codes = ['b', 'g', 'r', 'grey', 'm', 'orange']
    cmaps = ["Blues", "Greens", "Reds", "Greys", "Purples", "Oranges"]
    angle_points = 25

    plot_segments(segments, color_codes, angle_points)
    plot_segments_bins(segments_bins, color_codes, angle_points)
    plot_segments_bins_2D(segments_bins_2D, color_codes, angle_points)
    plot_segments_bins_2D_3D(segments_bins_2D, color_codes, angle_points, cmaps)
    plot_segments_bins_prototype_3D(segments_bins_prototype, color_codes, angle_points, cmaps)
    plot_ground_lines_3D(segments_bins_prototype, color_codes, angle_points, ground_lines)
    plot_segments_fitted(segments_bins_prototype, ground_lines, color_codes)

    plt.show()

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
    ax.set_zlim3d(-1, 1)
    return ax

def plot_data_2D(points):
    x = [coords[0] for coords in points]
    y = [coords[1] for coords in points]
    
    init_plot_2D("Point Cloud (2D)", "x", "y")
    plt.plot(x, y, '.', color='green')
    plt.plot(0, 0, 'o', color='black')
    angles = np.linspace(0, 2*math.pi, 100)
    plt.plot(LIDAR_RANGE*np.cos(angles), LIDAR_RANGE*np.sin(angles), color='black')
    # plt.savefig(FIGURES_DIR + "1_Point-Cloud-2D")

def plot_data_3D(points):
    x = [coords[0] for coords in points]
    y = [coords[1] for coords in points]
    z = [coords[2] for coords in points]

    ax = init_plot_3D("Point Cloud", "x", "y", "Height", 45, 45)
    ax.scatter3D(x, y, z, c=z, cmap='Greens');
    
    angles = np.linspace(0, 2*math.pi, 100)
    ax.plot3D(LIDAR_RANGE * np.cos(angles), LIDAR_RANGE * np.sin(angles), color='black')

    # plt.savefig(FIGURES_DIR + "2_Point-Cloud-3D")

def plot_segments(segments, color_codes, angle_points):
    init_plot_2D("Points assigned to Segments", "x", "y")
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
    # plt.savefig(FIGURES_DIR + "3_Segments")

def plot_segments_bins(segments_bins, color_codes, angle_points):
    init_plot_2D("Points assigned to Bins within Segments", "x", "y")

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
    # plt.savefig(FIGURES_DIR + "4_Bins-Segments")

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
    # plt.savefig(FIGURES_DIR + "5_2D-Approx-Point-Cloud-2D")

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
    # plt.savefig(FIGURES_DIR + "6_2D-Approx-Point-Cloud-3D")

def plot_segments_bins_prototype_3D(segments_bins_prototype, color_codes, angle_points, cmaps):
    ax = init_plot_3D("Prototype Points", "x", "y", "Height", 45, 45)

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
    # plt.savefig(FIGURES_DIR + "7_Prototype-Points")

def plot_ground_lines_3D(segments_bins_prototype, color_codes, angle_points, ground_lines):
    ax = init_plot_3D("Ground Plane Estimation", "x", "y", "Height", 45, 45)

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

    print("hey", ground_lines)

    for i in range(len(ground_lines)):
        for j in range(len(ground_lines[i])):
            start = ground_lines[i][j][2]
            end = ground_lines[i][j][3]
            r = np.linspace(start[0], end[0], 50)
            z = ground_lines[i][j][0] * r + ground_lines[i][j][1]
            x = r * math.cos((i + 0.5) * DELTA_ALPHA)
            y = r * math.sin((i + 0.5) * DELTA_ALPHA)
            ax.plot3D(x, y, z, color='black')

    # plt.savefig(FIGURES_DIR + "8_Ground-Plane-Estimation")

def plot_segments_fitted(segments_bins_prototype, ground_lines, color_codes):
    print("SP", segments_bins_prototype)
    print("--- Prototype Points ---")
    for i in range(len(segments_bins_prototype)):
        # This if statement is hacky. Without it, this visualisation function crashes
        if len(ground_lines[i]) > 0:
            color1 = color_codes[i % len(color_codes)]
            print("Segment:", i + 1)
            print(str(ground_lines[i][0][4]))
            init_plot_2D("Segment " + str(i + 1) + " | Degrees: " + str(round((i * DELTA_ALPHA) * 180/math.pi, 2)) + " to " + str(round((i + 1) * DELTA_ALPHA * 180/math.pi, 2)) + " | Points: " + str(ground_lines[i][0][4]), "Distance from origin", "Height")
            plt.ylim(-2, 2)
            x = []
            y = []
            for j in range(len(segments_bins_prototype[i])):
                if len(segments_bins_prototype[i][j]) != 0:
                    x.append(segments_bins_prototype[i][j][0])
                    y.append(segments_bins_prototype[i][j][1])
                    print("     Bin:", j, "Point:", segments_bins_prototype[i][j][0], segments_bins_prototype[i][j][1])
                    for k in range(len(ground_lines[i])):
                        origin = ground_lines[i][k][2][0]
                        end = ground_lines[i][k][3]
                        x_gp = np.linspace(origin, end[0], 50)
                        y_gp = ground_lines[i][k][0] * x_gp + ground_lines[i][k][1]
                        plt.plot(x_gp, y_gp, '--', color=color1)
            plt.plot(x, y, 'o', color='black')
            # plt.savefig(FIGURES_DIR + "9_Segment-" + str(i+1) + "-Fitted_Line")


# Max value of the norm of x and y (excluding z)
def init_points(points):
    global LIDAR_RANGE, NUM_BINS

    values = []
    for i in range(len(points)):
        values.append(math.sqrt(points[i][0] ** 2 + points[i][1] ** 2))
    
    
    if (math.pi % DELTA_ALPHA != 0):
        raise ValueError("Value of DELTA_ALPHA:", DELTA_ALPHA, "does not divide a circle into an integer-number of segments.")

    LIDAR_RANGE = math.ceil(max(values))
    print("Max x-y norm:", LIDAR_RANGE)

    NUM_BINS = math.ceil(LIDAR_RANGE / BIN_SIZE) # A derived constant

    plot_data_2D(points)
    plot_data_3D(points)
    ground_plane = get_ground_plane(points)
    print(ground_plane)

    time.sleep(15)
