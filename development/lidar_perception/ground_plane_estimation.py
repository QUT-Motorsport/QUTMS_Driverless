# - Modules -
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

def points_to_segment_2(points):
    segments = [[] for i in range(NUM_SEGMENTS)]
    for i in range(len(points)):
        point = points[i] # [x, y, z]
        seg_idx = get_segment(point[0], point[1])
        segments[seg_idx].append(point)
    return segments

# Creates a set of segments with bins
def init_bins():
    segments_bins = []
    for i in range(NUM_SEGMENTS):
        segments_bins.append([])
        for j in range(NUM_BINS):
            segments_bins[i].append([])
    #print("Number of bins:", len(segments_bins[0]))
    return segments_bins

# Returns true if the point (x, y) is in bin j
def in_bin(x, y, j):
    return (j*BIN_SIZE <= math.sqrt((x**2)+(y**2)) <= (j+1)*BIN_SIZE)

# Returns the index to a bin that a point (x, y) maps to 
def get_bin(x, y):
    norm = math.sqrt((x**2)+(y**2))
    bin_index = math.floor(norm / BIN_SIZE)
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

def points_to_bins_2(segments):
    segments_bins = [[[] for j in range(NUM_BINS)] for i in range(NUM_SEGMENTS)]
    for i in range(NUM_SEGMENTS):
        for j in range(len(segments[i])):
            point = segments[i][j] # [x, y, z]
            bin_index = get_bin(point[0], point[1])
            segments_bins[i][bin_index].append(point)
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

def approximate_2D_2(segments_bins):
    segments_bins_2D = []
    for i in range(NUM_SEGMENTS):
        segments_bins_2D.append([])
        for j in range(NUM_BINS):
            segments_bins_2D[i].append([])
            for k in range(len(segments_bins[i][j])):
                x = segments_bins[i][j][k][0]
                y = segments_bins[i][j][k][1]
                z = segments_bins[i][j][k][2]
                point_prime = [math.sqrt(x**2 + y**2), z]
                segments_bins_2D[i][j].append(point_prime)
            segments_bins_2D[i][j].sort(reverse=True)
    return segments_bins_2D

# Modifies input array
def approximate_2D_3(segments_bins):
    for i in range(NUM_SEGMENTS):
        for j in range(NUM_BINS):
            for k in range(len(segments_bins[i][j])):
                x = segments_bins[i][j][k][0]
                y = segments_bins[i][j][k][1]
                z = segments_bins[i][j][k][2]
                point_prime = [math.sqrt(x**2 + y**2), z]
                segments_bins[i][j][k] = point_prime
            segments_bins[i][j].sort(reverse=True)
    return segments_bins

# Modifies input array
def approximate_2D_4(segments_bins):
    for i in range(NUM_SEGMENTS):
        for j in range(NUM_BINS):
            for k in range(len(segments_bins[i][j])):
                point = segments_bins[i][j][k] # [x, y, z]
                point_prime = [math.sqrt(point[0]**2 + point[1]**2), point[2]]
                segments_bins[i][j][k] = point_prime
            segments_bins[i][j].sort(reverse=True)
    return segments_bins

# Modifies input array
def approximate_2D_5(segments_bins):
    for i in range(NUM_SEGMENTS):
        for j in range(NUM_BINS):
            for k in range(len(segments_bins[i][j])):
                point = segments_bins[i][j][k] # [x, y, z]
                point_prime = [math.sqrt(point[0]**2 + point[1]**2), point[2]]
                segments_bins[i][j][k] = point_prime
            segments_bins[i][j].sort(reverse=True)
            # Prototype points
            if len(segments_bins[i][j]) > 0:
                segments_bins[i][j] = segments_bins[i][j][0]
    return segments_bins

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

# Modifies input
def prototype_points_2(segments_bins_2D):
    for i in range(NUM_SEGMENTS):
        for j in range(NUM_BINS):
            if len(segments_bins_2D[i][j]) > 0:
                segments_bins_2D[i][j] = segments_bins_2D[i][j][0]
    return segments_bins_2D

# Not sure if this is working correctly
# Should compare 3D distance between points and start of line point
def dist_points_3D_old(x1, x2, m, b):
    x2_z = m*x2[0] + b
    distance = math.sqrt((x2[0] - x1[0])**2 + (x2[1] - x1[1])**2 + (x2_z - x1[2])**2)
    return distance

# distance from point to line in 3D space
# def point_line_dist_3D():
#    pass

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
def label_points_old(segments, ground_lines):
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
                closest_dist = dist_points_3D_old(point, line[2], line[0], line[1])
                for k in range(1, num_lines):
                    line = ground_lines[i][k]
                    dist_to_line = dist_points_3D_old(point, line[2], line[0], line[1])
                    if (dist_to_line < closest_dist):
                        closest_line = line
                        closest_dist = dist_to_line
                hacky_point = [math.sqrt(point[0]**2 + point[1]**2), point[2]] # fix this
                point_to_line_dist = line_extraction.dist_point_line(hacky_point, closest_line[0], closest_line[1])
                if (closest_dist < T_D_MAX and point_to_line_dist < T_D_GROUND):
                    is_ground = True
            labelled_points[i][j].append(is_ground)
    return labelled_points

# https://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
def dist_points_3D(x_0, x_1, x_2):
    numer = np.linalg.norm(np.cross(np.subtract(x_0, x_1), np.subtract(x_0, x_2)))
    denom = np.linalg.norm(np.subtract(x_2, x_1))
    distance = numer/denom
    return distance

def line_to_end_points(line, segment_idx):
    start = line[2] # First point in line
    end = line[3] # Last point in line
    r = np.linspace(start[0], end[0], 2)
    z = line[0] * r + line[1]
    x = r * math.cos((segment_idx + 0.5) * DELTA_ALPHA)
    y = r * math.sin((segment_idx + 0.5) * DELTA_ALPHA)
    x_1 = [x[0], y[0], z[0]]
    x_2 = [x[1], y[1], z[1]]
    return [x_1, x_2]

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
            num_lines = len(ground_lines[i])
            seg_idx = i
            # If there is ground line in corresponding segment
            # find the closest one
            if num_lines == 0:
                left_counter = i-1
                right_counter = i+1
                left_idx = (left_counter) % NUM_SEGMENTS
                right_idx = (right_counter) % NUM_SEGMENTS
                while left_idx != right_idx:
                    print("i:", i, "left:", left_idx, "right:", right_idx)
                    if len(ground_lines[left_idx]) > 0:
                        seg_idx = left_idx
                        break
                    elif len(ground_lines[right_idx]) > 0:
                        seg_idx = right_idx
                        break
                    left_counter -= 1
                    right_counter += 1
                    left_idx = (left_counter) % NUM_SEGMENTS
                    right_idx = (right_counter) % NUM_SEGMENTS
                if left_idx == right_idx:
                    raise AssertionError("No ground lines found")
            line = line_to_end_points(ground_lines[seg_idx][0], seg_idx)
            closest_dist = dist_points_3D(point, line[0], line[1])
            for k in range(1, num_lines):
                line = ground_lines[seg_idx][k]
                dist_to_line = dist_points_3D(point, line[0], line[1])
                if (dist_to_line < closest_dist):
                    closest_dist = dist_to_line
            #point_to_line_dist = dist_points_3D(point, closest_line)
            if (closest_dist < T_D_MAX and closest_dist < T_D_GROUND):
                is_ground = True
            labelled_points[i][j].append(is_ground)
    return labelled_points

# Conservative approach implemented using T_D_MAX parameter
# Modifies input
def label_points_2(segments, ground_lines):
    # Assuming multiple lines can be in one segment
    # Identifying closest line for each point
    for i in range(NUM_SEGMENTS):
        for j in range(len(segments[i])): 
            point = segments[i][j]
            is_ground = False
            num_lines = len(ground_lines[i])
            seg_idx = i
            # If there is no ground line in current segment, find the closest one
            if num_lines == 0:
                left_counter = i-1
                right_counter = i+1
                left_idx = (left_counter) % NUM_SEGMENTS
                right_idx = (right_counter) % NUM_SEGMENTS
                while left_idx != right_idx:
                    print("i:", i, "left:", left_idx, "right:", right_idx)
                    if len(ground_lines[left_idx]) > 0:
                        seg_idx = left_idx
                        break
                    elif len(ground_lines[right_idx]) > 0:
                        seg_idx = right_idx
                        break
                    left_counter -= 1
                    right_counter += 1
                    left_idx = (left_counter) % NUM_SEGMENTS
                    right_idx = (right_counter) % NUM_SEGMENTS
                if left_idx == right_idx:
                    raise AssertionError("No ground lines found")
            line = line_to_end_points(ground_lines[seg_idx][0], seg_idx)
            closest_dist = dist_points_3D(point, line[0], line[1])
            for k in range(1, num_lines):
                line = ground_lines[seg_idx][k]
                dist_to_line = dist_points_3D(point, line[0], line[1])
                if (dist_to_line < closest_dist):
                    closest_dist = dist_to_line
            if (closest_dist < T_D_MAX and closest_dist < T_D_GROUND):
                is_ground = True
            segments[i][j].append(is_ground)
    return segments

# Conservative approach implemented using T_D_MAX parameter
# Modifies input
def label_points_2_old(segments, ground_lines):
    # Assuming multiple lines in one segment
    # Identifying closest line for each point
    for i in range(NUM_SEGMENTS):
        # For every point in each segment. Perhaps make an array of constants of point counts?
        for j in range(len(segments[i])):
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
            segments[i][j].append(is_ground)
    return segments

def non_ground_points(labelled_points):
    # Flatten parent array (remove segements)
    labelled_points = [points for sublist in labelled_points for points in sublist]
    object_points = []
    for i in range(len(labelled_points)):
        point = labelled_points[i]
        if point[3] == False:
            object_points.append([point[0], point[1], point[2]])
    return object_points

def non_ground_points_2(labelled_points):
    # Flatten parent array (remove segements)
    labelled_points = [points for sublist in labelled_points for points in sublist]
    # Return all objects that are NOT flagged as ground
    return [point for point in labelled_points if point[3] == False]

# Ignoring height
def get_distance(point_a, point_b):
    # Distance
    return math.sqrt((point_b[0] - point_a[0])**2 + (point_b[1]-point_a[1])**2)

def object_reconstruction(cluster_centers, points):
    cone_width = 0.15
    reconstructed_clusters = []
    # You might want to sort points by x coords here
    # since that is how the clusters are sorted
    # This may reduce overall time
    for i in range (len(cluster_centers)):
        reconstructed_clusters.append([])
    for i in range(len(points)):
        point = points[i]
        for j in range(len(cluster_centers)):
            cluster_center = cluster_centers[j]
            # I'm gonna be hoping a point wont be in two clusters at the same time
            # therefore ill break after the first match for each point
            # Increases speed of algorithm
            if get_distance(cluster_center, point) <= cone_width:
                reconstructed_clusters[j].append(point)
                break;
    return reconstructed_clusters

def object_reconstruction_2(cluster_centers, points):
    cone_width = 0.15
    reconstructed_clusters = [[] for i in range(len(cluster_centers))]
    # You might want to sort points by x coords here
    # since that is how the clusters are sorted
    # This may reduce overall time
    for i in range(len(points)):
        point = points[i]
        for j in range(len(cluster_centers)):
            # I'm gonna be hoping a point wont be in two clusters at the same time
            # therefore ill break after the first match for each point
            # Increases speed of algorithm
            if get_distance(cluster_centers[j], point) <= cone_width:
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
    ERROR_MARGIN = 0.1 # Constant
    for i in range(len(reconstructed_clusters)):
        point_count = len(reconstructed_clusters[i])
        if point_count >= 1:
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
            #print(exp_point_count, point_count)
            if (exp_point_count*(1 - ERROR_MARGIN) <= point_count <= exp_point_count*(1 + ERROR_MARGIN)):
                cones.append([x_mean, y_mean])
    return cones

def benchmarking(points):
    print("here")
    # Optimised
    start_time = time.time()
    segments = points_to_segment_2(points)
    time_1 = time.time() - start_time

    # Optimised 
    start_time = time.time()
    segments_bins = points_to_bins_2(segments)
    time_2 = time.time() - start_time

    # Optimised
    #start_time = time.time()
    #segments_bins_2D = approximate_2D_4(segments_bins)
    #time_3 = time.time() - start_time

    # Optimised
    #start_time = time.time()
    #segments_bins_prototype = prototype_points_2(segments_bins_2D)
    #time_4 = time.time() - start_time

    # - Merging 2 Components -

    # Optimised: Modifies input array
    start_time = time.time()
    segments_bins_prototype = approximate_2D_5(segments_bins)
    time_3 = time.time() - start_time
    time_4 = 0 # Component Merged.

    start_time = time.time()
    ground_lines = line_extraction.get_ground_plane(segments_bins_prototype, NUM_SEGMENTS, NUM_BINS)
    time_5 = time.time() - start_time

    # Improved: Modifies input 
    # You need to re-optimise the new label points function
    start_time = time.time()
    labelled_points = label_points_2(segments, ground_lines)
    time_6 = time.time() - start_time

    # Optimised
    start_time = time.time()
    object_points = non_ground_points_2(labelled_points)
    time_7 = time.time() - start_time

    # Optimised? (should be quicker for larger array)
    start_time = time.time()
    #clusters, noise, cluster_centers = DBSCAN.init_DBSCAN(object_points)
    cluster_centers = DBSCAN.init_DBSCAN_2(object_points)
    time_8 = time.time() - start_time

    # Optimised (A tiny bit faster)
    start_time = time.time()
    reconstructed_clusters = object_reconstruction_2(cluster_centers, points)
    time_9 = time.time() - start_time

    # Already Optimised
    start_time = time.time()
    cones = get_cones(reconstructed_clusters)
    time_10 = time.time() - start_time

    return [time_1, time_2, time_3, time_4, time_5, time_6, time_7, time_8, time_9, time_10]

def get_ground_plane_old(points):
    # This is still using the unoptimised functions (for visualisation purposes)
    segments = points_to_segment(points)
    segments_bins = points_to_bins(segments)
    segments_bins_2D = approximate_2D_4(segments_bins)
    segments_bins_prototype = prototype_points(segments_bins_2D)
    ground_lines = line_extraction.extract_lines(segments_bins_prototype, NUM_SEGMENTS, NUM_BINS)
    # label_points labels ALL points, but don't we only need to append non-ground points (would speed up algorithm)
    labelled_points = label_points_2(segments, ground_lines)
    object_points = non_ground_points(labelled_points)
    clusters, noise, cluster_centers = DBSCAN.init_DBSCAN(object_points)
    reconstructed_clusters = object_reconstruction(cluster_centers, points)
    cones = get_cones(reconstructed_clusters)

    # Plotting
    # Could have plot after each thing above ^ and then have all of that
    # inside of a try() except with plt.show() guaranteed to occur.
    if VISUALISE : visualise_data(segments, segments_bins, segments_bins_2D, segments_bins_prototype, ground_lines, labelled_points, object_points, clusters, noise, reconstructed_clusters, cones)

    #print("\n\n\n", object_points)
    print(cones)
    return cones

def get_ground_plane(point_cloud):
    segments = points_to_segment_2(point_cloud)
    if VISUALISE: vis.plot_segments(segments)

    segments_bins = points_to_bins_2(segments)
    if VISUALISE: vis.plot_segments_bins(segments_bins, False)

    segments_bins_prototype = approximate_2D_5(segments_bins)

    ground_plane = line_extraction.get_ground_plane(segments_bins_prototype, NUM_SEGMENTS, NUM_BINS)
    if VISUALISE: vis.plot_ground_lines_3D(segments_bins_prototype, ground_plane, False)
    #if VISUALISE: vis.plot_segments_fitted(segments_bins_prototype, ground_plane)

    labelled_points = label_points_2(segments, ground_plane)
    if VISUALISE: vis.plot_labelled_points(labelled_points, ground_plane)

    object_points = non_ground_points_2(labelled_points)
    if VISUALISE: vis.plot_grid_2D(object_points)

    cluster_centers = DBSCAN.init_DBSCAN_2(object_points)

    reconstructed_clusters = object_reconstruction_2(cluster_centers, point_cloud)
    if VISUALISE: vis.plot_reconstruction(reconstructed_clusters)

    cones = get_cones(reconstructed_clusters)
    if VISUALISE: vis.plot_cones(cones)

    # Could consider try except block to ensure plotting - even during failure
    if VISUALISE and DISPLAY: plt.show() 
    return cones


def visualise_data(segments, segments_bins, segments_bins_2D, segments_bins_prototype, ground_lines, labelled_points, object_points, clusters, noise, reconstructed_clusters, cones):
    # Could add a try except here so I can always see the plots even if error occurs
    #vis.plot_data_2D()
    vis.plot_data_3D()
        #vis.plot_segments(segments)
        #vis.plot_segments_bins(segments_bins, False) # Laggy due to bin count (Set False to decrease lag)
    #vis.plot_segments_bins_2D(segments_bins_2D, False) # Laggy due to bin count (Set False to decrease lag)
    #vis.plot_segments_bins_2D_3D(segments_bins_2D, False) # Laggy due to bin count (Set False to decrease lag)
    #vis.plot_segments_bins_prototype_3D(segments_bins_prototype, False) # Laggy due to bin count (Set False to decrease lag)
        #vis.plot_ground_lines_3D(segments_bins_prototype, ground_lines, False) # Laggy due to bin count (Set False to decrease lag)
    #vis.plot_segments_fitted(segments_bins_prototype, ground_lines) # Creates many figures (One for each segment)
    vis.plot_labelled_points(labelled_points, ground_lines)
    vis.plot_grid_2D(object_points)
    vis.plot_clusters(clusters, noise)
    vis.plot_reconstruction(reconstructed_clusters)
    vis.plot_cones(cones)
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
    DELTA_ALPHA = 2*math.pi / 128 # Angle of each segment # 45 deg
    NUM_SEGMENTS = math.ceil(2*math.pi / DELTA_ALPHA) # Number of segments # 8
    BIN_SIZE = 1 # The length of a bin (in metres) # 1
    NUM_BINS = math.ceil(LIDAR_RANGE / BIN_SIZE) # A derived constant
    T_D_GROUND = 0.13 # Maximum distance between point and line to be considered part of ground plane. # 2
    T_D_MAX = 100 # Maximum distance a point can be from origin to even be considered for ground plane labelling. Otherwise it's automatically labelled as non-ground.
    
    if (math.pi % DELTA_ALPHA != 0):
        raise ValueError("Value of DELTA_ALPHA:", DELTA_ALPHA, "does not divide a circle into an integer-number of segments.")

def main(point_cloud, _visualise, _display, benchmark):
    init_constants()
    if benchmark:
        return benchmarking(point_cloud)
    else:
        global VISUALISE
        global DISPLAY
        VISUALISE = _visualise
        DISPLAY = _display
        if _display: VISUALISE = True

        # Remove this when a max range for the Lidar has been decided on
        global LIDAR_RANGE
        values = []
        for i in range(len(point_cloud)):
            values.append(math.sqrt(point_cloud[i][0] ** 2 + point_cloud[i][1] ** 2))
        LIDAR_RANGE = math.ceil(max(values)) 
        print("Max xy norm:", LIDAR_RANGE) # Max value of the norm of x and y (excluding z)

        if VISUALISE:
            FIGURES_DIR = "./development/lidar_perception/figures/"
            vis.init(point_cloud, DELTA_ALPHA, LIDAR_RANGE, BIN_SIZE, True, FIGURES_DIR)

        cones = get_ground_plane(point_cloud)
        return cones


#test_data = [[7.06342121e-07, -7.03119099e-01, -9.88163576e-02], [3.50690633e-02, -7.02601612e-01, -9.88676399e-02], [7.00787753e-02, -7.00345874e-01, -9.89192799e-02], [1.04956642e-01, -6.96342111e-01, -9.89711806e-02], [1.39609933e-01, -6.90619528e-01, -9.90231857e-02], [1.73955664e-01, -6.83158755e-01, -9.90751982e-02], [2.07900986e-01, -6.73988938e-01, -9.91270617e-02], [2.41362914e-01, -6.63129687e-01, -9.91786569e-02], [2.74258345e-01, -6.50610268e-01, -9.92298499e-02], [3.06504488e-01, -6.36460066e-01, -9.92805138e-02], [3.38018209e-01, -6.20698452e-01, -9.93305221e-02], [3.68721426e-01, -6.03374302e-01, -9.93797481e-02], [3.98530900e-01, -5.84536552e-01, -9.94280651e-02], [4.27373737e-01, -5.64214349e-01, -9.94753465e-02], [4.55181390e-01, -5.42456627e-01, -9.95214954e-02], [4.81870949e-01, -5.19331753e-01, -9.95663553e-02], [5.07378757e-01, -4.94878739e-01, -9.96098369e-02], [5.31641543e-01, -4.69175696e-01, -9.96518210e-02], [5.54590762e-01, -4.42261726e-01, -9.96922031e-02], [5.76168001e-01, -4.14234459e-01, -9.97308716e-02], [5.96319497e-01, -3.85142714e-01, -9.97677296e-02], [6.14991367e-01, -3.55064690e-01, -9.98026803e-02], [6.32135093e-01, -3.24078441e-01, -9.98356417e-02], [6.47706389e-01, -2.92252302e-01, -9.98665243e-02], [6.61656737e-01, -2.59683996e-01, -9.98952314e-02], [6.73956633e-01, -2.26441830e-01, -9.99217033e-02], [6.84571922e-01, -1.92613751e-01, -9.99458730e-02], [6.93473399e-01, -1.58277839e-01, -9.99676660e-02], [7.00636685e-01, -1.23531781e-01, -9.99870375e-02], [7.06041992e-01, -8.84732157e-02, -1.00003920e-01], [7.09679842e-01, -5.31802513e-02, -1.00018285e-01], [7.11530566e-01, -1.77408122e-02, -1.00030079e-01], [7.11594105e-01, 1.77474674e-02, -1.00039281e-01], [7.09875464e-01, 5.32064550e-02, -1.00045882e-01], [7.06369638e-01, 8.85189623e-02, -1.00049831e-01], [7.01086640e-01, 1.23616643e-01, -1.00051135e-01], [6.94045663e-01, 1.58411592e-01, -1.00049801e-01], [6.85256660e-01, 1.92796379e-01, -1.00045808e-01], [6.74749076e-01, 2.26702660e-01, -1.00039184e-01], [6.62551820e-01, 2.60032773e-01, -1.00029953e-01], [6.48689747e-01, 2.92698860e-01, -1.00018114e-01], [6.33201599e-01, 3.24622720e-01, -1.00003719e-01], [6.16126537e-01, 3.55726302e-01, -9.99868065e-02], [5.97513258e-01, 3.85921657e-01, -9.99674127e-02], [5.77410877e-01, 4.15130734e-01, -9.99455899e-02], [5.55863142e-01, 4.43285108e-01, -9.99213904e-02], [5.32938421e-01, 4.70316440e-01, -9.98948961e-02], [5.08685648e-01, 4.96156365e-01, -9.98661518e-02], [4.83173043e-01, 5.20736516e-01, -9.98352543e-02], [4.56464201e-01, 5.43988585e-01, -9.98022631e-02], [4.28627372e-01, 5.65873444e-01, -9.97672826e-02], [3.99740696e-01, 5.86313009e-01, -9.97303948e-02], [3.69877547e-01, 6.05278015e-01, -9.96917114e-02], [3.39111030e-01, 6.22709751e-01, -9.96513143e-02], [3.07519346e-01, 6.38569236e-01, -9.96093079e-02], [2.75185436e-01, 6.52817369e-01, -9.95658040e-02], [2.42192313e-01, 6.65424943e-01, -9.95209143e-02], [2.08627954e-01, 6.76352799e-01, -9.94747654e-02], [1.74570397e-01, 6.85591161e-01, -9.94274691e-02], [1.40112415e-01, 6.93101048e-01, -9.93791372e-02], [1.05337091e-01, 6.98872685e-01, -9.93298963e-02], [7.03371763e-02, 7.02906013e-01, -9.92798805e-02], [3.51956300e-02, 7.05181539e-01, -9.92292166e-02], [-2.00207452e-07, -9.86790895e-01, -9.87435356e-02], [4.92234081e-02, -9.86283302e-01, -9.88154635e-02], [9.83981565e-02, -9.83324528e-01, -9.88879427e-02], [1.47397190e-01, -9.77914512e-01, -9.89607945e-02], [1.96103215e-01, -9.70072925e-01, -9.90338475e-02], [2.44394168e-01, -9.59799588e-01, -9.91069227e-02], [2.92148113e-01, -9.47114229e-01, -9.91798267e-02], [3.39242786e-01, -9.32065606e-01, -9.92523804e-02], [3.85561198e-01, -9.14653659e-01, -9.93244052e-02], [4.30981159e-01, -8.94937038e-01, -9.93957147e-02], [4.75390345e-01, -8.72964501e-01, -9.94661227e-02], [5.18676400e-01, -8.48765433e-01, -9.95354727e-02], [5.60722470e-01, -8.22427690e-01, -9.96035561e-02], [6.01420641e-01, -7.93980598e-01, -9.96702164e-02], [6.40668809e-01, -7.63512015e-01, -9.97352824e-02], [6.78359270e-01, -7.31090248e-01, -9.97985825e-02], [7.14399397e-01, -6.96803272e-01, -9.98599455e-02], [7.48686492e-01, -6.60719395e-01, -9.99192223e-02], [7.81137705e-01, -6.22936308e-01, -9.99762490e-02], [8.11660349e-01, -5.83541870e-01, -1.00030877e-01], [8.40176105e-01, -5.42643547e-01, -1.00082971e-01], [8.66606772e-01, -5.00329196e-01, -1.00132383e-01], [8.90884399e-01, -4.56725806e-01, -1.00178987e-01], [9.12940085e-01, -4.11930948e-01, -1.00222655e-01], [9.32715654e-01, -3.66061896e-01, -1.00263268e-01], [9.50161994e-01, -3.19235831e-01, -1.00300737e-01], [9.65220690e-01, -2.71569848e-01, -1.00334935e-01], [9.77857709e-01, -2.23181188e-01, -1.00365788e-01], [9.88038599e-01, -1.74216390e-01, -1.00393206e-01], [9.95729148e-01, -1.24782830e-01, -1.00417115e-01], [1.00091469e+00, -7.50074685e-02, -1.00437455e-01], [1.00357604e+00, -2.50270106e-02, -1.00454167e-01], [1.00370800e+00, 2.50315797e-02, -1.00467212e-01], [1.00130594e+00, 7.50413537e-02, -1.00476541e-01], [9.96374428e-01, 1.24865584e-01, -1.00482143e-01], [9.88933027e-01, 1.74377337e-01, -1.00483999e-01], [9.78991866e-01, 2.23449618e-01, -1.00482099e-01], [9.66579974e-01, 2.71955550e-01, -1.00476444e-01], [9.51736450e-01, 3.19768071e-01, -1.00467071e-01], [9.34490323e-01, 3.66760314e-01, -1.00453980e-01], [9.14895833e-01, 4.12815064e-01, -1.00437231e-01], [8.93001616e-01, 4.57815140e-01, -1.00416847e-01], [8.68861139e-01, 5.01643360e-01, -1.00392886e-01], [8.42547715e-01, 5.44182539e-01, -1.00365430e-01], [8.14125121e-01, 5.85315406e-01, -1.00334533e-01], [7.83671081e-01, 6.24963999e-01, -1.00300290e-01], [7.51263976e-01, 6.62991464e-01, -1.00262791e-01], [7.16991663e-01, 6.99339211e-01, -1.00222133e-01], [6.80942118e-01, 7.33890116e-01, -1.00178421e-01], [6.43217623e-01, 7.66556323e-01, -1.00131787e-01], [6.03911161e-01, 7.97269285e-01, -1.00082338e-01], [5.63125074e-01, 8.25951159e-01, -1.00030214e-01], [5.20971894e-01, 8.52523506e-01, -9.99755561e-02], [4.77554083e-01, 8.76937747e-01, -9.99184921e-02], [4.32993650e-01, 8.99115622e-01, -9.98591930e-02], [3.87398094e-01, 9.19018149e-01, -9.97978002e-02], [3.40889424e-01, 9.36586618e-01, -9.97344777e-02], [2.93589681e-01, 9.51801717e-01, -9.96693969e-02], [2.45616123e-01, 9.64594841e-01, -9.96027067e-02], [1.97095782e-01, 9.74975824e-01, -9.95346084e-02], [1.48150533e-01, 9.82905686e-01, -9.94652510e-02], [9.89025086e-02, 9.88364875e-01, -9.93948281e-02], [4.94787581e-02, 9.91353333e-01, -9.93235037e-02], [2.58279852e-06, -1.64533591e+00, -9.85744894e-02], [8.21119472e-02, -1.64528739e+00, -9.86942574e-02], [1.64226100e-01, -1.64115679e+00, -9.88150835e-02], [2.46125430e-01, -1.63293445e+00, -9.89366472e-02], [3.27619463e-01, -1.62063968e+00, -9.90586653e-02], [4.08498406e-01, -1.60426307e+00, -9.91808325e-02], [4.88556981e-01, -1.58385324e+00, -9.93028432e-02], [5.67590117e-01, -1.55943942e+00, -9.94243771e-02], [6.45402670e-01, -1.53107071e+00, -9.95451510e-02], [7.21784472e-01, -1.49879551e+00, -9.96648446e-02], [7.96540439e-01, -1.46269226e+00, -9.97831449e-02], [8.69470119e-01, -1.42282927e+00, -9.98997465e-02], [9.40393031e-01, -1.37929428e+00, -1.00014366e-01], [1.00910413e+00, -1.33220470e+00, -1.00126676e-01], [1.07543218e+00, -1.28164828e+00, -1.00236394e-01], [1.13919699e+00, -1.22775209e+00, -1.00343227e-01], [1.20021772e+00, -1.17066252e+00, -1.00446887e-01], [1.25832808e+00, -1.11048698e+00, -1.00547090e-01], [1.31337714e+00, -1.04738164e+00, -1.00643575e-01], [1.36520827e+00, -9.81512606e-01, -1.00736082e-01], [1.41367042e+00, -9.13046002e-01, -1.00824341e-01], [1.45863628e+00, -8.42147708e-01, -1.00908130e-01], [1.49997950e+00, -7.68983841e-01, -1.00987211e-01], [1.53757226e+00, -6.93769097e-01, -1.01061344e-01], [1.57131279e+00, -6.16698980e-01, -1.01130344e-01], [1.60110807e+00, -5.37939250e-01, -1.01194017e-01], [1.62686503e+00, -4.57734287e-01, -1.01252176e-01], [1.64851558e+00, -3.76259774e-01, -1.01304665e-01], [1.66598666e+00, -2.93750077e-01, -1.01351328e-01], [1.67923391e+00, -2.10429803e-01, -1.01392038e-01], [1.68821371e+00, -1.26513824e-01, -1.01426676e-01], [1.69289160e+00, -4.22169678e-02, -1.01455137e-01], [1.69326305e+00, 4.22263853e-02, -1.01477362e-01], [1.68931794e+00, 1.26601398e-01, -1.01493262e-01], [1.68106639e+00, 2.10663915e-01, -1.01502806e-01], [1.66852748e+00, 2.94208854e-01, -1.01505965e-01], [1.65173566e+00, 3.77001882e-01, -1.01502731e-01], [1.63073993e+00, 4.58818346e-01, -1.01493105e-01], [1.60558891e+00, 5.39453268e-01, -1.01477131e-01], [1.57635570e+00, 6.18672132e-01, -1.01454832e-01], [1.54312360e+00, 6.96289420e-01, -1.01426281e-01], [1.50598514e+00, 7.72070765e-01, -1.01391569e-01], [1.46503830e+00, 8.45840454e-01, -1.01350784e-01], [1.42039990e+00, 9.17393327e-01, -1.01304047e-01], [1.37219214e+00, 9.86543894e-01, -1.01251483e-01], [1.32055199e+00, 1.05310655e+00, -1.01193257e-01], [1.26562047e+00, 1.11691535e+00, -1.01129517e-01], [1.20754933e+00, 1.17781401e+00, -1.01060450e-01], [1.14649498e+00, 1.23562670e+00, -1.00986242e-01], [1.08262801e+00, 1.29023623e+00, -1.00907110e-01], [1.01612437e+00, 1.34147668e+00, -1.00823261e-01], [9.47164595e-01, 1.38924062e+00, -1.00734942e-01], [8.75934422e-01, 1.43340099e+00, -1.00642391e-01], [8.02628756e-01, 1.47387004e+00, -1.00545861e-01], [6.10381126e-01, 1.26758587e+00, -8.42869729e-02], [5.35403013e-01, 1.27024269e+00, -8.25838149e-02], [4.71301526e-01, 1.29501832e+00, -8.25627521e-02], [4.62267071e-01, 1.49877524e+00, -9.39651951e-02], [4.11921799e-01, 1.61772883e+00, -1.00012936e-01], [3.30398530e-01, 1.63437951e+00, -9.98983011e-02], [2.48230666e-01, 1.64688957e+00, -9.97816697e-02], [1.65633038e-01, 1.65524924e+00, -9.96633470e-02], [8.28207135e-02, 1.65944874e+00, -9.95436385e-02], [1.95868324e-06, -4.89938831e+00, -9.77391526e-02], [2.45104179e-01, -4.91103983e+00, -9.80936512e-02], [4.91383284e-01, -4.91064024e+00, -9.84529555e-02], [7.38262892e-01, -4.89805365e+00, -9.88162681e-02], [4.15045381e-01, -2.05317426e+00, -4.17896062e-02], [1.23137784e+00, -4.83591604e+00, -9.95514393e-02], [1.47636294e+00, -4.78624916e+00, -9.99215096e-02], [1.71945381e+00, -4.72415972e+00, -1.00292005e-01], [1.96000552e+00, -4.64966869e+00, -1.00661978e-01], [2.19734955e+00, -4.56283426e+00, -1.01030439e-01], [2.43083096e+00, -4.46375322e+00, -1.01396412e-01], [2.65978122e+00, -4.35254240e+00, -1.01758890e-01], [2.88354111e+00, -4.22937965e+00, -1.02116868e-01], [3.10143685e+00, -4.09446764e+00, -1.02469303e-01], [3.31282377e+00, -3.94806242e+00, -1.02815203e-01], [3.51703310e+00, -3.79045558e+00, -1.03153504e-01], [3.71343446e+00, -3.62197971e+00, -1.03483208e-01], [3.90138459e+00, -3.44299626e+00, -1.03803284e-01], [4.08028603e+00, -3.25391483e+00, -1.04112759e-01], [2.80291390e+00, -2.01516151e+00, -6.88673928e-02], [2.91230369e+00, -1.88096261e+00, -6.91624805e-02], [4.55679655e+00, -2.63086963e+00, -1.04967698e-01], [3.89826417e+00, -1.99851942e+00, -8.73917267e-02], [4.14080381e+00, -1.86839402e+00, -9.06257853e-02], [4.93186665e+00, -1.93561709e+00, -1.05693229e-01], [5.03215504e+00, -1.69071507e+00, -1.05902374e-01], [5.11941671e+00, -1.44038343e+00, -1.06093906e-01], [5.19331884e+00, -1.18534470e+00, -1.06267162e-01], [5.25356388e+00, -9.26340997e-01, -1.06421515e-01], [5.29992151e+00, -6.64163351e-01, -1.06556438e-01], [5.33218336e+00, -3.99583459e-01, -1.06671408e-01], [5.35021639e+00, -1.33421421e-01, -1.06766038e-01], [5.35392332e+00, 1.33521900e-01, -1.06839970e-01], [5.34326029e+00, 4.00416464e-01, -1.06892928e-01], [5.31823683e+00, 6.66461468e-01, -1.06924728e-01], [5.27892637e+00, 9.30817068e-01, -1.06935248e-01], [5.22544098e+00, 1.19267261e+00, -1.06924467e-01], [4.04828072e+00, 1.13901043e+00, -8.38958323e-02], [3.89996839e+00, 1.31032014e+00, -8.20753276e-02], [4.98187399e+00, 1.95524180e+00, -1.06764972e-01], [2.84309649e+00, 1.28284800e+00, -6.22241609e-02], [4.75306702e+00, 2.43675637e+00, -1.06554858e-01], [4.61983109e+00, 2.66725492e+00, -1.06419697e-01], [4.47462177e+00, 2.89001894e+00, -1.06265090e-01], [4.31794262e+00, 3.10439444e+00, -1.06091604e-01], [4.15032148e+00, 3.30977607e+00, -1.05899848e-01], [3.97231483e+00, 3.50559688e+00, -1.05690487e-01], [3.78452229e+00, 3.69132042e+00, -1.05464265e-01], [3.58755541e+00, 3.86646771e+00, -1.05221950e-01], [3.38207293e+00, 4.03059387e+00, -1.04964375e-01], [3.16872430e+00, 4.18329954e+00, -1.04692400e-01], [2.94820714e+00, 4.32422400e+00, -1.04406945e-01], [2.72120571e+00, 4.45305729e+00, -1.04108930e-01], [2.48843288e+00, 4.56953764e+00, -1.03799328e-01], [2.25060630e+00, 4.67343044e+00, -1.03479125e-01], [5.58211446e-01, 1.32435811e+00, -2.86706518e-02], [1.76264358e+00, 4.84282160e+00, -1.02810897e-01], [1.51394272e+00, 4.90808630e+00, -1.02464914e-01], [1.26305461e+00, 4.96032810e+00, -1.02112398e-01], [1.01067698e+00, 4.99952841e+00, -1.01754360e-01], [7.57503510e-01, 5.02572012e+00, -1.01391822e-01], [5.04227579e-01, 5.03897810e+00, -1.01025820e-01], [2.51503170e-01, 5.03939438e+00, -1.00657322e-01], [4.18302417e-01, -2.06926608e+00, 4.21161875e-02], [3.96949291e+00, -2.03503680e+00, 8.89884979e-02], [3.99052310e+00, 1.34075296e+00, 8.39810446e-02], [2.86319089e+00, 1.29192293e+00, 6.26638457e-02], [5.62313974e-01, 1.33408964e+00, 2.88802125e-02], [4.21610475e-01, -2.08565044e+00, 1.27484918e-01], [2.88747764e+00, 1.30287349e+00, 1.89787671e-01], [5.66481411e-01, 1.34396791e+00, 8.73764530e-02], [5.70728660e-01, 1.35405159e+00, 1.47032723e-01]] 
test_data = [[0.50223053, -0.50223595, -0.09982084], [0.5127095, -0.49167976, -0.09983505], [0.52296877, -0.48089892, -0.09984906], [0.5329985, -0.46990329, -0.09986287], [0.54279882, -0.45870253, -0.09987646], [0.55235976, -0.44729671, -0.09988984], [0.56167668, -0.43568578, -0.099903, ], [0.57075441, -0.42388931, -0.09991594], [0.5795781, -0.41188774, -0.09992864], [0.58815283, -0.39971036, -0.0999411 ], [0.59646392, -0.38734743, -0.09995331], [0.6045211, -0.37482825, -0.09996527], [0.61230969, -0.36212352, -0.09997697], [0.61982983, -0.34926251, -0.09998842], [0.62707645, -0.33623549, -0.09999959], [0.63404965, -0.32307172, -0.1000105 ], [0.64074457, -0.30975172, -0.10002112], [0.64716125, -0.29629499, -0.10003147], [0.65328974, -0.28270149, -0.10004154], [0.65913016, -0.26899081, -0.1000513 ], [0.66468245, -0.25514337, -0.10006078], [0.66994184, -0.2411983, -0.10006995], [0.67490345, -0.227136, -0.10007882], [0.67957193, -0.21296629, -0.10008738], [0.68393785, -0.19869891, -0.10009563], [0.68800581, -0.18435338, -0.10010356], [0.69176626, -0.16991019, -0.10011118], [0.69522387, -0.15539862, -0.10011847], [0.69837385, -0.14081867, -0.10012544], [0.70121634, -0.12617037, -0.10013208], [0.70375115, -0.1114732, -0.10013838], [0.70597357, -0.09670767, -0.10014436], [0.70788324, -0.08190307, -0.10014999], [0.70948058, -0.06706915, -0.10015528], [0.71076536, -0.05219616, -0.10016023], [0.7117328, -0.03730363, -0.10016483], [0.71238786, -0.02239155, -0.10016908], [0.71273017, -0.00746969, -0.100173, ], [0.71275532, 0.00746195, -0.10017655], [0.71246296, 0.0223936, -0.10017975], [0.71185803, 0.03730574, -0.1001826 ], [0.71094078, 0.05219836, -0.10018509], [0.70970613, 0.06708122, -0.10018723], [0.70815897, 0.08193479, -0.10018901], [0.70629424, 0.09674933, -0.10019042], [0.70412219, 0.11151505, -0.10019148], [0.70163727, 0.12624173, -0.10019218], [0.6988402,  0.14090981, -0.10019252], [0.69573534, 0.15551932, -0.1001925 ], [0.69232774, 0.17005071, -0.10019213], [0.68860775, 0.184504, -0.10019138], [0.6845848,  0.19888894, -0.10019029], [0.68025929, 0.21317624, -0.10018884], [0.67563105, 0.22737566, -0.10018702], [0.67070973, 0.24146767, -0.10018485], [0.66548574, 0.25545228, -0.10018232], [0.65997362, 0.26932946, -0.10017943], [0.65416861, 0.28307971, -0.10017619], [0.64807063, 0.29671279, -0.10017259], [0.64168936, 0.31020916, -0.10016865], [0.63502496, 0.32355905, -0.10016436], [0.62808204, 0.33677223, -0.10015972], [0.62086105, 0.34983894, -0.10015474], [0.61336643, 0.36273968, -0.10014941], [0.60559845, 0.37548417, -0.10014374], [0.59756666, 0.38806263, -0.10013773], [0.58927131, 0.40046531, -0.10013139], [0.58071715, 0.41269228, -0.10012472], [0.57190919, 0.42473364, -0.10011772], [0.5628472,  0.43657976, -0.1001104 ], [0.55353612, 0.44824031, -0.10010274], [0.54398578, 0.45970556, -0.10009478], [0.53419149, 0.470956, -0.10008649], [0.52416754, 0.48199159, -0.1000779 ], [0.51391417, 0.49282211, -0.100069, ], [0.70665514, -0.70665878, -0.10000101], [0.72144103, -0.691845, -0.10002103], [0.73591918, -0.67670888, -0.10004079], [0.75007504, -0.66127986, -0.10006026], [0.76390392, -0.64555782, -0.10007943], [0.77740556, -0.62953293, -0.1000983 ], [0.79056525, -0.61322492, -0.10011686], [0.80337864, -0.59664345, -0.1001351 ], [0.81584507, -0.57979828, -0.10015301], [0.8279503, -0.56267971, -0.10017058], [0.83969909, -0.54530722, -0.10018782], [0.85107696, -0.52769053, -0.1002047 ], [0.86208361, -0.50983942, -0.10022122], [0.87271428, -0.49175394, -0.10023737], [0.88295919, -0.47344384, -0.10025314], [0.89281833, -0.45491889, -0.10026854], [0.90228194, -0.43618882, -0.10028354], [0.91135508, -0.41725367, -0.10029815], [0.9200229, -0.39813298, -0.10031236], [0.92828542, -0.37882671, -0.10032615], [0.93614262, -0.35935438, -0.10033953], [0.94358498, -0.33971602, -0.1003525 ], [0.95060724, -0.3199214, -0.10036502], [0.95721447, -0.29998028, -0.10037712], [0.96340203, -0.27990243, -0.10038877], [0.96915972, -0.25968784, -0.10039999], [0.9744876, -0.23935604, -0.10041074], [0.97938609, -0.21892656, -0.10042104], [0.98385471, -0.19837989, -0.10043089], [0.98788393, -0.17775506, -0.10044027], [0.9914785, -0.15703256, -0.10044918], [0.99463367, -0.13625145, -0.10045762], [0.99734944, -0.11540196, -0.10046557], [0.9996208, -0.09449385, -0.10047305], [1.00144804, -0.07354667, -0.10048004], [1.00283551, -0.0525604, -0.10048655], [1.00377393, -0.03154482, -0.10049257], [1.00427294, -0.01051945, -0.1004981 ], [1.00432265, 0.0105157, -0.10050312], [1.00392807, 0.03154111, -0.10050765], [1.00308418, 0.05256654, -0.10051168], [1.00180101, 0.07356268, -0.1005152 ], [1.00006843, 0.09452955, -0.10051822], [0.99789655, 0.11545738, -0.10052074], [0.99528033, 0.13632663, -0.10052274], [0.99221969, 0.15714706, -0.10052424], [0.9887194,  0.17788941, -0.10052522], [0.98478496, 0.19856341, -0.10052571], [0.98041087, 0.21913977, -0.10052568], [0.97559714, 0.23962826, -0.10052515], [0.97035873, 0.2599996, -0.10052411], [0.96468574, 0.28026351, -0.10052256], [0.95858312, 0.3004005, -0.1005205 ], [0.95205581, 0.32040074, -0.10051793], [0.94510847, 0.3402546, -0.10051485], [0.93774098, 0.35996193, -0.10051128], [0.92996365, 0.37951306, -0.1005072 ], [0.92176622, 0.39887863, -0.10050261], [0.91316348, 0.41807824, -0.10049754], [0.9041605,  0.43708253, -0.10049196], [0.89475214, 0.45590129, -0.10048589], [0.8849532,  0.47450525, -0.10047933], [0.87475854, 0.49289435, -0.10047229], [0.8641783,  0.51107848, -0.10046476], [0.8532173,  0.52900863, -0.10045674], [0.84188008, 0.54672396, -0.10044826], [0.83017182, 0.56418538, -0.10043931], [0.81809741, 0.58139294, -0.10042988], [0.80566651, 0.59833682, -0.10041999], [0.79287416, 0.61500728, -0.10040963], [0.77973539, 0.63141412, -0.10039882], [0.76625955, 0.64752799, -0.10038757], [0.7524417,  0.66336846, -0.10037587], [0.73829204, 0.67889637, -0.10036372], [0.72382498, 0.69411182, -0.10035115], [1.18524253, -1.18524408, -0.10042281], [1.21021438, -1.16055763, -0.10045654], [1.23465884, -1.13533413, -0.10048983], [1.25857604, -1.10958326, -0.10052264], [1.28195107, -1.08333421, -0.10055496], [1.30476964, -1.05658722, -0.10058678], [1.32702172, -1.02935183, -0.10061809], [1.34870255, -1.00163794, -0.10064886], [1.36979759, -0.97346497, -0.10067909], [1.39029181, -0.94484276, -0.10070876], [1.41018081, -0.9157812, -0.10073785], [1.42945457, -0.88629961, -0.10076635], [1.44810319, -0.85640776, -0.10079426], [1.46611726, -0.82611567, -0.10082155], [1.48348677, -0.79544252, -0.10084821], [1.50020695, -0.76439834, -0.10087423], [1.51626766, -0.73299277, -0.1008996 ], [1.53165972, -0.70125514, -0.1009243 ], [1.5463779, -0.66918552, -0.10094833], [1.56041741, -0.63680333, -0.10097167], [1.57376349, -0.60410857, -0.1009943 ], [1.58641136, -0.57114035, -0.10101623], [1.59836113, -0.53791827, -0.10103744], [1.60960281, -0.50442266, -0.10105792], [1.62013161, -0.47069266, -0.10107765], [1.62993765, -0.43674779, -0.10109663], [1.63902116, -0.40258801, -0.10111485], [1.64737713, -0.36823291, -0.1011323 ], [1.65500081, -0.33371174, -0.10114899], [1.66188717, -0.29902455, -0.10116487], [1.66803598, -0.26419082, -0.10117997], [1.67343783, -0.22923014, -0.10119427], [1.67809761, -0.19417174, -0.10120777], [1.68200541, -0.15899613, -0.10122044], [1.68516111, -0.12375213, -0.1012323 ], [1.68756974, -0.08844949, -0.10124333], [1.68922186, -0.05308825, -0.10125353], [1.69011676, -0.01770743, -0.10126289], [1.69025993, 0.01770272, -0.10127141], [1.68964636, 0.05309337, -0.10127909], [1.68827569, 0.08847429, -0.10128591], [1.68615317, 0.12381618, -0.1012919 ], [1.68327868, 0.15910925, -0.10129702], [1.67964745, 0.19434379, -0.10130129], [1.67526424, 0.2294807, -0.1013047 ], [1.67013395, 0.26451999, -0.10130724], [1.66425657, 0.29944208, -0.10130892], [1.65763223, 0.33422753, -0.10130973], [1.65026534, 0.36887631, -0.10130969], [1.64216638, 0.40334928, -0.10130878], [1.63332987, 0.43764654, -0.10130701], [1.62376106, 0.47174853, -0.10130437], [1.61347449, 0.50563574, -0.10130088], [1.60246027, 0.53928858, -0.10129652], [1.59073317, 0.57269728, -0.10129131], [1.57829821, 0.60585207, -0.10128524], [1.56515491, 0.63872379, -0.10127832], [1.55131376, 0.67131227, -0.10127055], [1.53678381, 0.70358825, -0.10126194], [1.52157021, 0.73555177, -0.1012525 ], [1.50568318, 0.76718336, -0.10124221], [1.48912179, 0.79845363, -0.10123109], [1.47189665, 0.82937229, -0.10121914], [1.45402217, 0.85990041, -0.10120638], [1.43550277, 0.89004773, -0.1011928 ], [1.41634893, 0.9197849, -0.10117842], [1.39656508, 0.94910216, -0.10116324], [1.37616599, 0.97798979, -0.10114727], [1.35515642, 1.00642812, -0.10113051], [1.33355117, 1.0344075, -0.10111298], [1.31136012, 1.06191826, -0.10109467], [1.28858781, 1.08894074, -0.1010756 ], [1.2652539,  1.11546516, -0.10105579], [1.24136341, 1.14149153, -0.10103524], [1.2169261,  1.16699064, -0.10101396], [3.6360724, -3.63607311, -0.10258278], [3.71525073, -3.5627954, -0.10268858], [3.79290533, -3.48775959, -0.10279309], [3.86899757, -3.41098547, -0.10289627], [3.94347787, -3.33250237, -0.10299806], [4.01631355, -3.25234938, -0.10309839], [4.08746004, -3.17056513, -0.10319722], [2.81251407, -2.08876395, -0.06988766], [2.80280781, -1.99185908, -0.06859463], [2.83580852, -1.92722189, -0.06839928], [2.89610791, -1.8807596, -0.06888841], [4.416502, -2.73835301, -0.10366707], [4.47670555, -2.64751577, -0.10375582], [4.53496647, -2.55533075, -0.10384271], [3.89774299, -2.08996749, -0.08822905], [3.89801717, -1.98615873, -0.08727434], [3.89827132, -1.88451803, -0.08637733], [4.07728291, -1.86674321, -0.08945779], [4.795856, -2.07536149, -0.10424767], [4.84173965, -1.97588134, -0.10432246], [4.88545465, -1.87534618, -0.10439507], [4.92697716, -1.77382445, -0.10446545], [4.96627855, -1.67134571, -0.10453359], [5.00334406, -1.56794882, -0.10459945], [5.03813839, -1.4637121, -0.10466295], [5.07064819, -1.35867441, -0.1047241 ], [5.10085249, -1.25289452, -0.10478286], [5.12872362, -1.1464113, -0.10483916], [5.15425539, -1.03928351, -0.104893, ], [5.17742348, -0.9315697, -0.10494435], [5.19821405, -0.8233186, -0.10499316], [5.21661234, -0.714589, -0.1050394 ], [5.23260832, -0.60543925, -0.10508308], [5.24618769, -0.49590865, -0.10512413], [5.25734472, -0.38608491, -0.10516254], [5.26606512, -0.27598765, -0.1051983 ], [5.27234411, -0.16569495, -0.10523137], [5.27618265, -0.05525566, -0.10526176], [5.27756834, 0.05526185, -0.10528942], [5.27650404, 0.16581853, -0.10531435], [5.2729888,  0.27633622, -0.10533654], [5.26701212, 0.38678572, -0.10535595], [5.25858927, 0.49707904, -0.10537259], [5.24771452, 0.6071772, -0.10538646], [5.23439932, 0.71702152, -0.10539752], [5.21864176, 0.82655346, -0.10540579], [5.20045805, 0.93571436, -0.10541126], [5.17985296, 1.04444575, -0.10541392], [5.15683508, 1.15268886, -0.10541377], [5.13141918, 1.26039505, -0.10541081], [5.10362148, 1.36751533, -0.10540505], [3.94076633, 1.14486599, -0.08186541], [3.89994025, 1.22213352, -0.08153111], [3.89979672, 1.312397, -0.08208491], [3.90701222, 1.40657628, -0.08283879], [4.92940617, 1.89222026, -0.10533433], [2.99134517, 1.22071195, -0.06445242], [2.85976672, 1.23750532, -0.06216225], [2.8961997,  1.32595062, -0.06354408], [2.89639258, 1.40013981, -0.06417757], [4.69837809, 2.39393926, -0.10519467], [4.64561844, 2.49095297, -0.10515863], [4.59073973, 2.58674645, -0.10511993], [4.53378057, 2.6812706, -0.1050786 ], [4.47476196, 2.77446628, -0.10503466], [4.41372108, 2.8662951, -0.10498814], [4.35068703, 2.95672774, -0.10493906], [4.28570652, 3.0456953, -0.10488746], [4.21880579, 3.13315916, -0.10483336], [4.15002108, 3.21907997, -0.10477678], [4.07939959, 3.30342889, -0.10471778], [4.00697136, 3.38615656, -0.10465639], [3.9327805,  3.46721458, -0.10459262], [3.85686612, 3.54657316, -0.10452653], [3.77927184, 3.62419391, -0.10445815], [2.8668282, -2.03735638, 0.07016182], [2.88427687, -1.96016014, 0.06956881], [3.96808577, -2.02185249, 0.08884356], [4.02117491, -1.94392514, 0.08910098], [5.62894392, 1.50829697, 0.11625462], [3.97838688, 1.24671984, 0.08317166], [3.99094582, 1.34306705, 0.08400393], [2.87567282, 1.24437976, 0.06250827], [2.86830306, 1.3131789,  0.06293224], [2.90354991, -1.97326243, 0.2103236 ], [4.00569344, -2.04102063, 0.26934198], [4.004879, 1.25502014, 0.25144288], [2.89451098, 1.25253379, 0.18895359], [2.90084004, 1.32808089, 0.19114089], [2.923172, -1.98659861, 0.35366076], [2.91368818, 1.26083481, 0.31768471]]

#test_data = [[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0], [4, 0, 0], [5, 0, 0], [6, 0, 0.1], [7, 0, 1.11], [8, 0, 2.10], [9, 0, 3.11], [10, 0, 4.10]]

print(len(test_data))
import time

start_time = time.time()

main(test_data, False, True, False)

print("\n Total Time: %s seconds " % (time.time() - start_time))



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
#   - bruh I was about to write this exact comment ^ create a constant for 
#     the number of points and reference it instead. 
#   - I should consider sorting the points early on, I think multiple
#     parts of the program would benefit by this / already need to do this
#   - Program encounters issues if point is at EXACT xy origin, because what
#     segment is this even in? This is practically impossible to encounter however.
#   - Update len() - 1 notation to [-1] where applicable.