# Modules
import time
import math
import numpy as np

# Plotting Data
import matplotlib.pyplot as plt

# Line Fitting 
from . import line_extraction 
 
# Point Cloud Clustering 
from . import DBSCAN 
 
# Visualiser 
from . import visualiser as vis 

# Returns the index to a segment that a point maps to
def get_segment(x, y):
    return math.floor(math.atan2(y, x) / DELTA_ALPHA)


# Creates the set of segments for points
def init_segments():
    segments = [] 
    for i in range(NUM_SEGMENTS):
        segments.append([])
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
    # If a point REALLY exceeds the limits, it shouldn't be included
    # Since processing points based on range will be done at the start,
    # this shouldn't be needed since the number of bins is calculated 
    # based on the specified range. Thus, by this point, all points should
    # belong a bin
    if bin_index >= NUM_BINS:
        bin_index = -1
        #print("Point exceeds expected max range of LIDAR. bin_index:", bin_index)
    return math.floor(bin_index)


def points_to_bins_2(segments):
    segments_bins = [[[] for j in range(NUM_BINS)] for i in range(NUM_SEGMENTS)]
    for i in range(NUM_SEGMENTS):
        for j in range(len(segments[i])):
            point = segments[i][j] # [x, y, z]
            bin_index = get_bin(point[0], point[1])
            if bin_index != -1:
                segments_bins[i][bin_index].append(point)
    return segments_bins


# Does not modify input array
def approximate_2D_6(segments_bins):
    segments_approx = [[[] for j in range(NUM_BINS)] for i in range(NUM_SEGMENTS)]
    for i in range(NUM_SEGMENTS):
        for j in range(NUM_BINS):
            for k in range(len(segments_bins[i][j])):
                point = segments_bins[i][j][k] # [x, y, z]
                point_prime = [math.sqrt(point[0]**2 + point[1]**2), point[2]]
                segments_approx[i][j].append(point_prime)
            segments_approx[i][j].sort(reverse=True)
            # Prototype points
            if len(segments_approx[i][j]) > 0:
                segments_approx[i][j] = segments_approx[i][j][0]
    return segments_approx


# Modifies input
def prototype_points_2(segments_bins_2D):
    for i in range(NUM_SEGMENTS):
        for j in range(NUM_BINS):
            if len(segments_bins_2D[i][j]) > 0:
                segments_bins_2D[i][j] = segments_bins_2D[i][j][0]
    return segments_bins_2D


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
# Modifies input
def label_points_4(segments_bins, ground_lines):
    # Assuming multiple lines can be in one segment
    # Identifying closest line for each point
    for i in range(NUM_SEGMENTS):
        for j in range(len(segments_bins[i])):
            for k in range(len(segments_bins[i][j])):
                point = segments_bins[i][j][k]
                is_ground = True
                num_lines = len(ground_lines[i])
                seg_idx = i
                # If there is no ground line in current segment, find the closest one
                if num_lines == 0:
                    left_counter = i-1
                    right_counter = i+1
                    left_idx = (left_counter) % NUM_SEGMENTS
                    right_idx = (right_counter) % NUM_SEGMENTS
                    while left_idx != right_idx:
                        #print("i:", i, "left:", left_idx, "right:", right_idx)
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
                ground_line = ground_lines[seg_idx][0]
                line_height = ground_line[0] * j + ground_line[1]
                if point[2] > line_height + 0.08: # Make this a constant
                    is_ground = False
                #     line = line_to_end_points(ground_line, seg_idx) 
                #     closest_dist = dist_points_3D(point, line[0], line[1]) 
                #     for m in range(1, num_lines): 
                #         line = ground_lines[seg_idx][m] 
                #         dist_to_line = dist_points_3D(point, line[0], line[1]) 
                #         if (dist_to_line < closest_dist): 
                #             closest_dist = dist_to_line 
                #     # bin_index + 2 is extra leeway 
                #     dynamic_T_D_GROUND = 4*(BIN_SIZE*(j))*math.tan(DELTA_ALPHA/2) # Solved for gradient of segment wrt points and distance 
                #     print("closest, dynamic:", closest_dist, dynamic_T_D_GROUND)
                #     if (closest_dist < T_D_MAX and closest_dist < dynamic_T_D_GROUND): 
                #         is_ground = True 

                segments_bins[i][j][k].append(is_ground)
    return segments_bins


# def label_points_4(segments_bins, ground_lines): 
#     # Assuming multiple lines can be in one segment 
#     # Identifying closest line for each point 
#     for i in range(NUM_SEGMENTS): 
#         for j in range(len(segments_bins[i])): 
#             for k in range(len(segments_bins[i][j])): 
#                 point = segments_bins[i][j][k] 
#                 is_ground = False 
#                 num_lines = len(ground_lines[i]) 
#                 seg_idx = i 
#                 # If there is no ground line in current segment, find the closest one 
#                 if num_lines == 0: 
#                     left_counter = i-1 
#                     right_counter = i+1 
#                     left_idx = (left_counter) % NUM_SEGMENTS 
#                     right_idx = (right_counter) % NUM_SEGMENTS 
#                     while left_idx != right_idx: 
#                         #print("i:", i, "left:", left_idx, "right:", right_idx) 
#                         if len(ground_lines[left_idx]) > 0: 
#                             seg_idx = left_idx 
#                             break 
#                         elif len(ground_lines[right_idx]) > 0: 
#                             seg_idx = right_idx 
#                             break 
#                         left_counter -= 1 
#                         right_counter += 1 
#                         left_idx = (left_counter) % NUM_SEGMENTS 
#                         right_idx = (right_counter) % NUM_SEGMENTS 
#                     if left_idx == right_idx: 
#                         raise AssertionError("No ground lines found") 
#                 ground_line = ground_lines[seg_idx][0] 
#                 line_height = ground_line[0] * j + ground_line[1] 
#                 if point[2] < line_height + 0.08: # Make this a constant 
#                     line = line_to_end_points(ground_line, seg_idx) 
#                     closest_dist = dist_points_3D(point, line[0], line[1]) 
#                     for m in range(1, num_lines): 
#                         line = ground_lines[seg_idx][m] 
#                         dist_to_line = dist_points_3D(point, line[0], line[1]) 
#                         if (dist_to_line < closest_dist): 
#                             closest_dist = dist_to_line 
#                     # bin_index + 2 is extra leeway 
#                     dynamic_T_D_GROUND = 4*(BIN_SIZE*(j))*math.tan(DELTA_ALPHA/2) # Solved for gradient of segment wrt points and distance 
#                     print("closest, dynamic:", closest_dist, dynamic_T_D_GROUND)
#                     if (closest_dist < T_D_MAX and closest_dist < dynamic_T_D_GROUND): 
#                         is_ground = True 
#                 segments_bins[i][j][k].append(is_ground) 
#     return segments_bins 
 

def non_ground_points_2(labelled_points):
    # Flatten parent array (remove bins)
    labelled_points = [points for sublist in labelled_points for points in sublist]
    # Flatten parent array (remove segements)
    labelled_points = [points for sublist in labelled_points for points in sublist]
    # Return all objects that are NOT flagged as ground
    return [point for point in labelled_points if point[3] == False]


# Ignoring height
def get_distance(point_a, point_b):
    # Distance
    return math.sqrt((point_b[0] - point_a[0])**2 + (point_b[1]-point_a[1])**2)


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
                break
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
    ERROR_MARGIN = 0.2 # Constant
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
            else:
                cones.append([x_mean, y_mean]) # Remove when real-er data
    return cones


def benchmarking(point_cloud):
    # might be able to modifiy segments directly if label points doesn't need it
    start_time = time.time()
    segments = points_to_segment_2(point_cloud)
    print("points_to_segment", time.time() - start_time)

    if VISUALISE: vis.plot_segments(segments)

    start_time = time.time()
    segments_bins = points_to_bins_2(segments)
    print("points_to_bins", time.time() - start_time)
    
    if VISUALISE: vis.plot_segments_bins(segments_bins, False)

    start_time = time.time()
    segments_bins_prototype = approximate_2D_6(segments_bins)
    print("approximate_2D", time.time() - start_time)

    start_time = time.time()
    ground_plane = line_extraction.get_ground_plane(segments_bins_prototype, NUM_SEGMENTS, NUM_BINS)
    print("get_ground_plane", time.time() - start_time)

    if VISUALISE: vis.plot_ground_lines_3D(segments_bins_prototype, ground_plane, False)
    #if VISUALISE: vis.plot_segments_fitted(segments_bins_prototype, ground_plane)

    start_time = time.time()
    labelled_points = label_points_4(segments_bins, ground_plane)
    print("label_points", time.time() - start_time)

    if VISUALISE: vis.plot_labelled_points(labelled_points, ground_plane)

    start_time = time.time()
    object_points = non_ground_points_2(labelled_points)
    print("non_ground_points", time.time() - start_time)

    if VISUALISE: vis.plot_grid_2D(object_points)

    start_time = time.time()
    cluster_centers = DBSCAN.get_objects(object_points)
    print("get_objects", time.time() - start_time)

    start_time = time.time()
    reconstructed_clusters = object_reconstruction_2(cluster_centers, point_cloud)
    print("object_reconstruction", time.time() - start_time)

    if VISUALISE: vis.plot_reconstruction(reconstructed_clusters)

    start_time = time.time()
    cones = get_cones(reconstructed_clusters)
    print("get_cones", time.time() - start_time)

    if VISUALISE: vis.plot_cones(cones)

    # Could consider try except block to ensure plotting - even during failure
    plt.show() 
    return cones


def get_ground_plane(point_cloud):
    # might be able to modifiy segments directly if label points doesn't need it
    segments = points_to_segment_2(point_cloud)
    if VISUALISE: vis.plot_segments(segments)

    segments_bins = points_to_bins_2(segments)
    if VISUALISE: vis.plot_segments_bins(segments_bins, False)

    segments_bins_prototype = approximate_2D_6(segments_bins)
    ground_plane = line_extraction.get_ground_plane(segments_bins_prototype, NUM_SEGMENTS, NUM_BINS)
    if VISUALISE: vis.plot_ground_lines_3D(segments_bins_prototype, ground_plane, False)
    #if VISUALISE: vis.plot_segments_fitted(segments_bins_prototype, ground_plane)

    labelled_points = label_points_4(segments_bins, ground_plane)
    if VISUALISE: vis.plot_labelled_points(labelled_points, ground_plane)

    object_points = non_ground_points_2(labelled_points)
    if VISUALISE: vis.plot_grid_2D(object_points)
    
    cluster_centers = DBSCAN.get_objects(object_points)
    reconstructed_clusters = object_reconstruction_2(cluster_centers, point_cloud)
    if VISUALISE: vis.plot_reconstruction(reconstructed_clusters)

    cones = get_cones(reconstructed_clusters)
    if VISUALISE: vis.plot_cones(cones)

    # Could consider try except block to ensure plotting - even during failure
    # if VISUALISE and DISPLAY: plt.show() 
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
    LIDAR_RANGE = 32 # Max range of the LIDAR # 100 # in metres
    DELTA_ALPHA = 2*math.pi / 128 # Angle of each segment # 45 deg
    NUM_SEGMENTS = math.ceil(2*math.pi / DELTA_ALPHA) # Number of segments # 8
    BIN_SIZE = 0.25 # The length of a bin (in metres) # 1
    NUM_BINS = math.ceil(LIDAR_RANGE / BIN_SIZE) # A derived constant

    T_D_GROUND = 0.1 # Maximum distance between point and line to be considered part of ground plane. # 2
    # T_D_PREV = 3           # Max distance of the first point of a line to the line previously fitted

    T_D_MAX = 100 # Maximum distance a point can be from origin to even be considered for ground plane labelling. Otherwise it's automatically labelled as non-ground.
    
    if (math.pi % DELTA_ALPHA != 0):
        raise ValueError("Value of DELTA_ALPHA:", DELTA_ALPHA, "does not divide a circle into an integer-number of segments.")

def lidar_main(point_cloud, _visualise, _display, benchmark, _figures_dir):
    init_constants()
    global VISUALISE
    global DISPLAY
    VISUALISE = _visualise
    DISPLAY = _display
    if _display: VISUALISE = True

    # Remove this when a max range for the Lidar has been decided on
    global LIDAR_RANGE
    
    # print("Max xy norm:", LIDAR_RANGE) # Max value of the norm of x and y (excluding z)

    if VISUALISE:
        FIGURES_DIR = _figures_dir
        vis.init_constants(point_cloud, DELTA_ALPHA, LIDAR_RANGE, BIN_SIZE, VISUALISE, FIGURES_DIR)
    
    # values = []
    # for i in range(len(point_cloud)):
    #    values.append(math.sqrt(point_cloud[i][0] ** 2 + point_cloud[i][1] ** 2))
    # LIDAR_RANGE = math.ceil(max(values))

    if benchmark:
        cones = benchmarking(point_cloud)
        return cones

    else:        
        cones = get_ground_plane(point_cloud)
        return cones

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

# 7. T_D_PREV has been updated to a dynamic mathematical relation that increases
#    as points get further from the origin. HOWEVER, T_D_PREV is used in
#    ground_plane_estimation, not here anymore. 