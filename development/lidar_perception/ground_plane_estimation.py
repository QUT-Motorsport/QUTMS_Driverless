# - Modules -
from development.lidar_perception.visualiser import ANGLE_RESOLUTION
import math
import copy

# Plotting Data
from mpl_toolkits.mplot3d import axes3d
import numpy as np
import matplotlib.pyplot as plt
from sklearn import cluster

# Line Fitting
import line_extraction

# Point Cloud Clustering
import DBSCAN

# Visualiser
import visualiser as vis

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

# m
# b
# first point
# last point
# num points

# Can there be multiple lines PER segment?? I still don't know and this will
# directly affect how this function is written. 
# But assuming that there is multiple lines ...

# Conservative approach implemented using T_D_MAX parameter
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
                point_to_line_dist = line_extraction.dist_point_line(hacky_point, closest_line[0], closest_line[1])
                if (closest_dist < T_D_MAX and point_to_line_dist < T_D_GROUND):
                    is_ground = True
            labelled_points[i][j].append(is_ground)
    return labelled_points

def non_ground_points(labelled_points):
    # Flatten parent array (remove segements)
    labelled_points = [points for sublist in labelled_points for points in sublist]

    object_points = []
    for i in range(len(labelled_points)):
        point = labelled_points[i]
        if point[3] == False:
            object_points.append([point[0], point[1], point[2]])
    return object_points

# Ignoring height
def get_distance(point_a, point_b):
    distance = math.sqrt((point_b[0] - point_a[0])**2 + (point_b[1]-point_a[1])**2)
    return distance

def object_reconstruction(cluster_centers, points):
    cone_width = 0.15
    reconstructed_clusters = []
    # You might want to sort points by x coords here
    # since that is how the clusters are sorted
    # This may reduce overall time
    for i in range (len(cluster_centers)):
        reconstructed_clusters.append([])

    for i in range(len(points)):
        point  = points[i]
        for j in range(len(cluster_centers)):
            cluster_center = cluster_centers[j]
            # I'm gonna be hoping a point wont be in two clusters at the same time
            # therefore ill break after the first match for each point
            if get_distance(cluster_center, point) <= cone_width:
                reconstructed_clusters[j].append(point)
                break;

    return reconstructed_clusters

# I NEED TO COMPUTE THE CENTER OF A CLUSTER ONLY ONCE
# AND KEEP THIS VALUE. Instead of calculating it multiple times.
def cone_filter(distance):
    cone_height = 0.325
    cone_width = 0.228
    horizontal_res = 0.2 * (math.pi / 180) # 0.2 degrees in between each point
    vertical_res = 2 * (math.pi / 180) # 2 degrees in between each point
    exp_point_count = (1/2) * (cone_height / (2 * distance * math.tan(vertical_res / 2))) * (cone_width / (2 * distance * math.tan(horizontal_res / 2)))
    return exp_point_count

def get_cones(reconstructed_clusters):
    cones = []
    error_margin = 0.1
    for i in range(len(reconstructed_clusters)):
        point_count = len(reconstructed_clusters[i])

        x_cluster = [coords[0] for coords in reconstructed_clusters[i]]
        y_cluster = [coords[1] for coords in reconstructed_clusters[i]]
        # Univserity of melbourne used z as well
        #z_cluster = [coords[1] for coords in reconstructed_clusters[i]]
        x_mean  = sum(x_cluster) / len(x_cluster)
        y_mean  = sum(y_cluster) / len(y_cluster)
        #z_mean  = sum(y_cluster) / len(y_cluster)
        distance = math.sqrt(x_mean ** 2 + y_mean ** 2)

        # Rule based filter
        exp_point_count = cone_filter(distance)
        print(exp_point_count, point_count)
        if (exp_point_count*(1 - error_margin) <= point_count <= exp_point_count*(1 + error_margin)):
            cones.append([x_mean, y_mean])
    return cones


def get_ground_plane(points):
    segments = points_to_segment(points)
    segments_bins = points_to_bins(segments)
    segments_bins_2D = approximate_2D(segments_bins)
    segments_bins_prototype = prototype_points(segments_bins_2D)
    ground_lines = line_extraction.extract_lines(segments_bins_prototype, NUM_SEGMENTS, NUM_BINS)
    labelled_points = label_points(segments, ground_lines)
    object_points = non_ground_points(labelled_points)
    clusters, noise, cluster_centers = DBSCAN.init_DBSCAN(object_points)
    reconstructed_clusters = object_reconstruction(cluster_centers, points)
    # f*ck that felt nice to finally type: (cones = )
    cones = get_cones(reconstructed_clusters)
    print("\n\n\nAHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH\n")
    print(cones)

    #print("\n\n\n", object_points)
    if VISUALISE : visualise_data(segments, segments_bins, segments_bins_2D, segments_bins_prototype, ground_lines, labelled_points, object_points, clusters, noise, reconstructed_clusters)
    return cones

def visualise_data(segments, segments_bins, segments_bins_2D, segments_bins_prototype, ground_lines, labelled_points, object_points, clusters, noise, reconstructed_clusters):
    vis.plot_segments(segments)
    vis.plot_segments_bins(segments_bins)
    vis.plot_segments_bins_2D(segments_bins_2D)
    vis.plot_segments_bins_2D_3D(segments_bins_2D)
    vis.plot_segments_bins_prototype_3D(segments_bins_prototype)
    vis.plot_ground_lines_3D(segments_bins_prototype, ground_lines)
    vis.plot_segments_fitted(segments_bins_prototype, ground_lines)
    vis.plot_labelled_points(labelled_points, ground_lines)
    vis.plot_grid_2D(object_points)
    vis.plot_clusters(clusters, noise)
    vis.plot_reconstruction(reconstructed_clusters)
    plt.show()

# new_x.append(norm[k] * math.cos((i + 0.5) * DELTA_ALPHA))
# new_y.append(norm[k] * math.sin((i + 0.5) * DELTA_ALPHA))

def init_constants():
    global LIDAR_RANGE
    global DELTA_ALPHA
    global NUM_SEGMENTS
    global BIN_SIZE
    global NUM_BINS
    global T_D_GROUND
    global T_D_MAX
    
    # Constants
    LIDAR_RANGE = 150 # Max range of the LIDAR # 100 # in metres
    DELTA_ALPHA = 2*math.pi / 64 # Angle of each segment # 45 deg
    NUM_SEGMENTS = math.ceil(2*math.pi / DELTA_ALPHA) # Number of segments # 8
    BIN_SIZE = 1 # The length of a bin (in metres) # 1
    NUM_BINS = math.ceil(LIDAR_RANGE / BIN_SIZE) # A derived constant
    T_D_GROUND = 1 # Maximum distance between point and line to be considered part of ground plane. # 2
    T_D_MAX = 100 # Maximum distance a point can be from origin to even be considered for ground plane labelling. Otherwise it's automatically labelled as non-ground.
    
def main(point_cloud, _visualise):
    global VISUALISE
    VISUALISE = _visualise

    init_constants()

    # Remove this when a max range for the Lidar has been decided on
    global LIDAR_RANGE
    values = []
    for i in range(len(point_cloud)):
        values.append(math.sqrt(point_cloud[i][0] ** 2 + point_cloud[i][1] ** 2))
    LIDAR_RANGE = math.ceil(max(values)) 
    print("Max x-y norm:", LIDAR_RANGE) # Max value of the norm of x and y (excluding z)
    # --------------------------------------------------------------

    cones = get_ground_plane(point_cloud)

    if VISUALISE:
        FIGURES_DIR = "./development/lidar_perception/figures/"
        vis.init(point_cloud, DELTA_ALPHA, LIDAR_RANGE, BIN_SIZE, True, FIGURES_DIR)
        vis.plot_data_2D(point_cloud)
        vis.plot_data_3D(point_cloud)

    return cones

#print(ground_plane)

# p1 = [1, 1.5]
# p2 = [2, 2]
# p3 = [3, 2.5]
# p4 = [4, 3]
# p5 = [5, 3.5]
# test_line = [p1, p2, p3, p4, p5]
# ground_lines = line_extraction.extract_lines([test_line], 1, 5)
# print("Line:", ground_lines)

# Notes
#   - Explore Python threading for this entire process
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
#   - T_D_GROUND: Perhaps this should be dynamic and scale with the 
#     size of bins / size of segments, rather than being static. Same
#     goes for T_D_MAX. Maybe it should be a lil dynamic too
#   - Points that constructed the ground plane lines should already be
#     labelled as ground, rather than being relablled. 
#   - num_points = len(points) # make this contanst above if its used #     a lot throughout code