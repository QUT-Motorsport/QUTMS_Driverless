# Modules
import time
import math
from typing import List
import matplotlib.pyplot as plt
import logging

RUN_ROS = True
if RUN_ROS:
    # Line Fitting
    from . import line_extraction
    # Point Cloud Clustering
    from . import DBSCAN
    # Visualiser
    from . import visualiser as vis
else:
    # Line Fitting
    import line_extraction
    # Point Cloud Clustering
    import DBSCAN
    # Visualiser
    import visualiser as vis


# typed way of referencing value in list
X = 0
Y = 1
Z = 2
I = 3

# Returns the index to a bin that a point (x, y) maps to 
def get_bin(x: float, y: float) -> int:
    # distance
    norm = math.sqrt((x**2)+(y**2))
    # which bin this distance falls in
    bin_index = math.floor(norm / BIN_SIZE)
    if norm % BIN_SIZE != 0: # <----- idk why this is here
        bin_index += 1
    if bin_index >= NUM_BINS: # out of range of LiDAR
        bin_index = -1
    return math.floor(bin_index)


# Returns the index to a segment that a point maps to
def get_segment(x: float, y: float) -> int:
    # which segment this angle falls in
    return math.floor(math.atan2(y, x) / DELTA_ALPHA)


def points_to_seg_bin(point_cloud: List[List]) -> List[List[List]]:
    # create segments[bins[points[]]]
    segments_bins: List[List[List]] = [[[] for j in range(NUM_BINS)] for i in range(NUM_SEGMENTS)]
    for i in range(len(point_cloud)): # iterate through each point
        point = point_cloud[i] # current point
        if point[Z] <= 100 and point[X] > 0: # only take points in front 180 degrees

            bin_idx = get_bin(point[X], point[Y])
            if bin_idx != -1: # 
                seg_idx: int = get_segment(point[X], point[Y])
                segments_bins[seg_idx][bin_idx].append(
                    [point[X], point[Y], point[Z]]
                )
    return segments_bins


# Does not modify input array
def approximate_2D(segments_bins: List[List[List]]) -> List[List[List]]:
    # create segments[bins[points[]]]
    segments_approx: List[list] = [[[] for j in range(NUM_BINS)] for i in range(NUM_SEGMENTS)]
    for i in range(NUM_SEGMENTS):
        for j in range(NUM_BINS):
            # find prototype points
            for k in range(len(segments_bins[i][j])):
                point: list = segments_bins[i][j][k] # [x, y, z]
                point_prime: list = [ (math.sqrt(point[X]**2 + point[Y]**2)), point[Z] ]
                segments_approx[i][j].append(point_prime)
            segments_approx[i][j].sort(reverse=True)
            # if more than 1 prototype point, take the first in the sorted array
            if len(segments_approx[i][j]) > 0:
                segments_approx[i][j] = segments_approx[i][j][0]
    return segments_approx


# https://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
def dist_points_3D(x_0: list, p_1: list, p_2: list) -> float:
    # get distance in each dimension from x to point 1
    p_1_dist = [ (x_0[X] - p_1[X]), (x_0[Y] - p_1[Y]), (x_0[Z] - p_1[Z]) ]
    # get distance in each dimension from x to point 2
    p_2_dist = [ (x_0[X] - p_2[X]), (x_0[Y] - p_2[Y]), (x_0[Z] - p_2[Z]) ]
    # cross product of dimension distances
    dist_cross = [
        p_1_dist[Y]*p_2_dist[Z] - p_2_dist[Y]*p_1_dist[Z], 
        -(p_1_dist[X]*p_2_dist[Z] - p_2_dist[X]*p_1_dist[Z]), 
        p_1_dist[X]*p_2_dist[Y] - p_2_dist[X]*p_1_dist[Y]
    ]
    # normalise (pythag) each cross
    dist_norm: float = math.sqrt(dist_cross[X]**2 + dist_cross[Y]**2 + dist_cross[Z]**2)
    # normalise point 1 and 2 distances
    p_norm: float = math.sqrt(
        (p_2[X] - p_1[X])**2 + (p_2[Y] - p_1[Y])**2 + (p_2[Z] - p_1[Z])**2
    )
    # return distance
    return dist_norm / p_norm


# Returns the start and end points (x, y, z) of a line
def line_to_end_points(line: List[List], segment_idx: int):
    start = line[2] # First point in line
    end = line[3] # Last point in line
    
    x_1 = start[0] * math.cos((segment_idx + 0.5) * DELTA_ALPHA)
    x_2 = end[0] * math.cos((segment_idx + 0.5) * DELTA_ALPHA)
    
    y_1 = start[0] * math.sin((segment_idx + 0.5) * DELTA_ALPHA)
    y_2 = end[0] * math.sin((segment_idx + 0.5) * DELTA_ALPHA)
    
    p_1 = [x_1, y_1, start[Y]]
    p_2 = [x_2, y_2, end[Y]]
    
    return [p_1, p_2]


# Modifies input
# Conservative approach implemented using T_D_MAX parameter
def label_points(segments_bins: List[List[List]], ground_lines: List[List[List]]):
    for i in range(NUM_SEGMENTS):
        num_lines = len(ground_lines[i])
        seg_idx = i
        # If there is no ground line in current segment, find the closest one
        if num_lines == 0:
            left_counter = i-1
            right_counter = i+1
            left_idx = (left_counter) % NUM_SEGMENTS
            right_idx = (right_counter) % NUM_SEGMENTS
            while left_idx != right_idx:
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
        for j in range(NUM_BINS):
            for k in range(len(segments_bins[i][j])):
                point = segments_bins[i][j][k]
                is_ground = False
                line_height = ground_line[0] * (j * BIN_SIZE) + ground_line[1]
                if point[2] < line_height + 0.10: # Make this a constant
                    line = line_to_end_points(ground_line, seg_idx)
                    closest_dist = dist_points_3D(point, line[0], line[1])
                    for m in range(1, num_lines):
                        ground_line = ground_lines[seg_idx][m]
                        line = line_to_end_points(ground_line, seg_idx)
                        dist_to_line = dist_points_3D(point, line[0], line[1])
                        if (dist_to_line < closest_dist):
                            closest_dist = dist_to_line
                    dynamic_T_D_GROUND = 2*((j + 1) * BIN_SIZE)*math.tan(DELTA_ALPHA/2) + BIN_SIZE * 1.3 # Solved for gradient of segment wrt points and distance
                    if (closest_dist < T_D_MAX and closest_dist < dynamic_T_D_GROUND):
                        is_ground = True
                segments_bins[i][j][k].append(is_ground)
    return segments_bins


# Modifies input
# def label_points_6(segments_bins, ground_lines):
#     for i in range(NUM_SEGMENTS):
#         num_lines = len(ground_lines[i])
#         seg_idx = i
#         # If there is no ground line in current segment, find the closest one:
#         if num_lines == 0:
#             left_counter = i-1
#             right_counter = i+1
#             left_idx = (left_counter) % NUM_SEGMENTS
#             right_idx = (right_counter) % NUM_SEGMENTS
#             while left_idx != right_idx:
#                 if len(ground_lines[left_idx]) > 0:
#                     seg_idx = left_idx
#                     break
#                 elif len(ground_lines[right_idx]) > 0:
#                     seg_idx = right_idx
#                     break
#                 left_counter -= 1
#                 right_counter += 1
#                 left_idx = (left_counter) % NUM_SEGMENTS
#                 right_idx = (right_counter) % NUM_SEGMENTS
#             if left_idx == right_idx: raise AssertionError("No ground lines found") # probably shouldnt have an error thrown 
#         ground_line = ground_lines[seg_idx][0]
                
#         for j in range(NUM_BINS):
#             avg_point = [0, 0, 0]
#             is_ground = False
#             for k in range(len(segments_bins[i][j])):
#                 avg_point[X] += segments_bins[i][j][k][X]
#                 avg_point[Y] += segments_bins[i][j][k][Y]
#                 avg_point[Z] += segments_bins[i][j][k][Z]
                
#                 segments_bins[i][j][k].append(is_ground)

#             line = line_to_end_points(ground_line, seg_idx)
#             closest_dist = dist_points_3D(avg_point, line[0], line[1])
            
#             for k in range(1, num_lines):
#                 line = ground_lines[seg_idx][k]
#                 dist_to_line = dist_points_3D(avg_point, line[0], line[1])
#                 if (dist_to_line < closest_dist):
#                     closest_dist = dist_to_line
#     return segments_bins


def non_ground_points(labelled_points: List[List[List]]) -> List[List[List]]:
    # Flatten parent array (remove bins)
    labelled_points = [points for sublist in labelled_points for points in sublist]
    # Flatten parent array (remove segements)
    labelled_points = [points for sublist in labelled_points for points in sublist]
    # Return all objects that are NOT flagged as ground
    return [point for point in labelled_points if point[3] == False]


# Ignoring height
def get_distance(point_a: List, point_b: List) -> float:
    # Distance
    return math.sqrt((point_b[X] - point_a[X])**2 + (point_b[Y]-point_a[Y])**2)


def count_nearby_segs(bin_idx: int, object_width: float) -> float:
    norm: float = (bin_idx + 1) * BIN_SIZE
    seg_length: float = norm * math.tan(DELTA_ALPHA / 2)
    return object_width / seg_length # nearby segments

def count_nearby_bins(object_width):
    return object_width / BIN_SIZE


# I NEED TO COMPUTE THE CENTER OF A CLUSTER ONLY ONCE
# AND KEEP THIS VALUE. Instead of calculating it multiple times.
#HORIZONTAL_RES = 0.192 * (math.pi / 180) # 0.384 degrees in between each point
HORIZONTAL_RES = 0.05 * (math.pi / 180) # 0.384 degrees in between each point
VERTICAL_RES = 1.25 * (math.pi / 180) # 1.25 degrees in between each point

CONE_HEIGHT = 0.3 #m
CONE_WIDTH = 0.15 #m

def object_reconstruction(cluster_centers, segments_bins, ground_lines) -> List[List]:
    ERROR_MARGIN = 1.2
    object_check_radius = 0.6 / 2
    cone_radius = 0.15 / 2 * ERROR_MARGIN
    reconstructed_clusters = []
    bins_to_check = math.ceil(count_nearby_bins(object_check_radius) / 2)
    for i in range(len(cluster_centers)):
        bad_boys: List = [] # questionable naming
        good_boys: List = []
        cluster = cluster_centers[i]
        
        # Find matching ground line
        # If z_mean of cluster is too close to the ground, skip to next cluster
        seg = get_segment(cluster[X], cluster[Y])
        
        # reused some code from above
        num_lines = len(ground_lines[seg])
        seg_idx = seg
        if num_lines == 0:
            left_counter = seg-1
            right_counter = seg+1
            left_idx = (left_counter) % NUM_SEGMENTS
            right_idx = (right_counter) % NUM_SEGMENTS
            while left_idx != right_idx:
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
        
        bin = get_bin(cluster[X], cluster[Y])
        
        line_height = ground_line[0] * (bin * BIN_SIZE) + ground_line[1]
        # make sure the height of each cluster is near the height of a cone
        if cluster[Z] >= line_height + (CONE_HEIGHT / 2) * 0.8 and cluster[Z] <= line_height + (CONE_HEIGHT / 2) * 1.2: # Make this a constant
            reconstructed_clusters.append([])
            seg_idx = seg
            bin_idx = bin
            segs_to_check = math.ceil(count_nearby_segs(bin_idx, object_check_radius) / 2)
            # print("for loop", seg_idx - segs_to_check, (seg_idx + segs_to_check + 1) % NUM_SEGMENTS)
            min_seg = seg_idx - segs_to_check
            if min_seg < 0:
                min_seg = NUM_SEGMENTS + min_seg
            max_seg = (seg_idx + segs_to_check + 1) % NUM_SEGMENTS
            if min_seg > max_seg:
                temp_min = min_seg
                min_seg = max_seg
                max_seg = temp_min
            for j in range(min_seg, max_seg):
                min_bin = bin_idx - bins_to_check
                if min_bin < 0:
                    min_bin = 0
                max_bin = bin_idx + bins_to_check + 1
                if max_bin > NUM_BINS:
                    max_bin = NUM_BINS
                for k in range(min_bin, max_bin):
                    for m in range(len(segments_bins[j][k])):
                        point = segments_bins[j][k][m]
                        distance = get_distance(cluster, point)
                        if distance <= object_check_radius:
                            if point[3] == False or distance <= cone_radius:
                                # Fix this, increment a counter to keep track of length
                                reconstructed_clusters[len(reconstructed_clusters)-1].append(point)
                                good_boys.append(point)
                        else:
                            bad_boys.append(point)
        #vis.plot_bad_boys(cluster, bad_boys, good_boys, segs_to_check)

    return reconstructed_clusters

	
def cone_filter(distance: float) -> float:
    #if distance < BIN_SIZE:
    #    distance = BIN_SIZE
    return (1/2) * (CONE_HEIGHT / (2*distance*math.tan(VERTICAL_RES/2))) * (CONE_WIDTH / (2*distance*math.tan(HORIZONTAL_RES/2)))


def newtons_method(x, x1, y1):
    x_2 = x * x
    x_5 = x_2 * x_2 * x
    x_6 = x_5 * x
    return (x * (12*A_2 - 8*A*x_2*y1 + x_5*x1)) / (10*A_2 - 6*A*x_2*y1 + x_6)

def F_1(x):
    return 1.05*x - 4.675

def F_2(x):
    return 1.05*x - 3.7

def F_3(x):
    return 1.05*x - 7.3

# A = 307.7506
A = 1181.7633
A_2 = A * A
def new_cone_filter(distance: float, point_count: int) -> bool:
    ERROR_MARGIN = 0.95
    
    if point_count >= F_3(distance):
        x_0 = math.sqrt(A/point_count)
    else:
        x_0 = distance
        
    x_n = x_0
    iterations = 10
    for i in range(iterations):
        x_n = newtons_method(x_n, distance, point_count)
        # NEED TO DO CHECK HERE. IF LAST ONE EQUALS NEW ONE THEN BREAK
        
    closest_point = cone_filter(x_n)
    dist = math.sqrt((x_n - distance)**2 + (closest_point - point_count)**2)
    # print(distance, point_count, x_n, closest_point, dist)
    # and x_n >= distance * ERROR_MARGIN and closest_point * ERROR_MARGIN <= point_count
    # and closest_point <= point_count * 1.5
    if dist <= 2.5 and x_n >= distance * 0.85:
        return True
    else:
        return False
    

FAR_X = 60 #m
def get_cones(reconstructed_clusters: List[List]) -> List[List]:
    cones: List[List] = []
    ERROR_MARGIN = 0.20 # Constant
    for i in range(len(reconstructed_clusters)):
        point_count = len(reconstructed_clusters[i])
        if point_count >= 1:
            x_cluster = [coords[X] for coords in reconstructed_clusters[i]]
            y_cluster = [coords[Y] for coords in reconstructed_clusters[i]]
            # Univserity of melbourne used z as well
            #z_cluster = [coords[1] for coords in reconstructed_clusters[i]]
            x_mean  = sum(x_cluster) / len(x_cluster)
            y_mean  = sum(y_cluster) / len(y_cluster)
            #z_mean  = sum(y_cluster) / len(y_cluster)
            distance = math.sqrt(x_mean ** 2 + y_mean ** 2)
            
            # only checks centre of scan for cones - noise filter (delete if needed)
            if abs(x_mean) < FAR_X:
                # print("    ", x_mean, y_mean, point_count, distance)
                if (new_cone_filter(distance, point_count)):
                    cones.append([x_mean, y_mean])
    return cones


def get_ground_plane(point_cloud: List[List]) -> List[List]:
    # might be able to modifiy segments directly if label points doesn't need it
    start_time: float = time.time()
    now: float = time.time()
    segments_bins: List[List[List]] = points_to_seg_bin(point_cloud)
    print("points_to_seg_bin", time.time() - now)
    
    if VISUALISE: vis.plot_segments_bins(segments_bins, False)

    now = time.time()
    segments_bins_prototype: List[List[List]] = approximate_2D(segments_bins)
    print("approximate_2D", time.time() - now)

    now = time.time()
    print(len(point_cloud))
    ground_plane: List[List[List]] = line_extraction.get_ground_plane(segments_bins_prototype, NUM_SEGMENTS, NUM_BINS)
    print("get_ground_plane", time.time() - now)

    if VISUALISE: vis.plot_ground_lines_3D(segments_bins_prototype, ground_plane, False)
    #if VISUALISE: vis.plot_segments_fitted(segments_bins_prototype, ground_plane)

    now = time.time()
    labelled_points: List[List[List]] = label_points(segments_bins, ground_plane)
    print("label_points", time.time() - now)

    if VISUALISE: vis.plot_labelled_points(labelled_points, ground_plane)

    now = time.time()
    object_points: List[List[List]] = non_ground_points(labelled_points)
    print("non_ground_points", time.time() - now)

    if VISUALISE: vis.plot_grid_2D(object_points)

    cones: List = []
    if len(object_points) > 0:
        now = time.time()
        cluster_centers = DBSCAN.get_objects(object_points)
        print("get_objects", time.time() - now)

        now = time.time()
        reconstructed_clusters: List[List] = object_reconstruction(cluster_centers, segments_bins, ground_plane)
        print("object_reconstruction", time.time() - now)

        if VISUALISE: vis.plot_reconstruction(reconstructed_clusters)

        now = time.time()
        cones: List[List] = get_cones(reconstructed_clusters)
        print("get_cones", time.time() - now)

        if VISUALISE: vis.plot_cones(cones)

    # Could consider try except block to ensure plotting - even during failure
    if VISUALISE and DISPLAY: plt.show()
    
    print("Algorithm Time:", time.time() - start_time)

    return cones


def lidar_init(_visualise: bool, _display: bool, _figures_dir: str, _max_range: int=16):
    global LIDAR_RANGE
    global DELTA_ALPHA
    global NUM_SEGMENTS
    global BIN_SIZE
    global NUM_BINS
    global T_D_GROUND
    global T_D_MAX

    global VISUALISE
    global DISPLAY
    global FIGURES_DIR

    # Constants
    LIDAR_RANGE = _max_range # Max range of the LIDAR (m)
    DELTA_ALPHA = 2*math.pi / 128 # Angle of each segment # 2*pi / 64 implies 64 segments
    NUM_SEGMENTS = math.ceil(2*math.pi / DELTA_ALPHA) # Number of segments # 8
    BIN_SIZE = 0.14 # The length of a bin (in metres) # 1
    NUM_BINS = math.ceil(LIDAR_RANGE / BIN_SIZE) # A derived constant

    T_D_GROUND = 0.1 # Maximum distance between point and line to be considered part of ground plane. # 2
    # T_D_PREV = 3           # Max distance of the first point of a line to the line previously fitted

    T_D_MAX = 100 # Maximum distance a point can be from origin to even be considered for ground plane labelling. Otherwise it's automatically labelled as non-ground.
    
    if (math.pi % DELTA_ALPHA != 0):
        raise ValueError("Value of DELTA_ALPHA:", DELTA_ALPHA, "does not divide a circle into an integer-number of segments.")
    
    VISUALISE = _visualise
    DISPLAY = _display
    if _display: VISUALISE = True
    FIGURES_DIR = _figures_dir


def lidar_main(point_cloud: List[List]):
    if VISUALISE:
        vis.init_constants(point_cloud, DELTA_ALPHA, LIDAR_RANGE, BIN_SIZE, VISUALISE, FIGURES_DIR)

    return get_ground_plane(point_cloud)

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
# 8. Implement logging
# 9. Consider removing completly empty segments at the start and see if that works!!!
#    I might still be treating this like I have a full circle of vision when really
#    It would only be 180 degrees
# 10. Investigate potential 'hacky point fix' in label_points function. I think this
#     has definitely been resolved though.
# 11. Review the non_ground_points function for the double flattening and see if 
#     this can be simpler. 
# 12. Object Reconstruction
# # You might want to sort points by x coords here
    # since that is how the clusters are sorted
    # This may reduce overall time
    # I'm gonna be hoping a point wont be in two clusters at the same time
            # therefore ill break after the first match for each point
            # Increases speed of algorithm

# Consider having the TM_SMALL simply has negative T_M, since we may have a line
# that slopes slightly downwards. But right now all angles below 0 degress are being
# filtered out

# Need to create a mathematical relationship between how many segments to check for points
# that may belong to a cluster vs. how far away the cluster is from the origin. Need to 
# consider the diameter of a cone. and mathy the bin sizs w.r.t. cone size

# bin_idx + 1 or j+1 since if a point is near the origin and in bin 0, you want it to have
# a norm from origin of at least BIN_SIZE, not 0.

# ADD AN OUTER BOUND to the desmos graph and any objects within this larger range can be 
# the 'unknown' cones that AMZ shows in their demo. Perhaps if the zed camera sees a cone 
# that matches with an unknown cone then WA-BAM NEW CONE PHASES INTO EXISTENCE

# Figure out the angle of the point cloud you've been working with and compare it to how
# how much you needed to change the max anlgle by. Create a linear line and using the
# angle of the car from the IMU you should be able to dynamically set this max angle for
# ground lines value.

# I wouldn't mind making the ground plane an object. It would be much MUCH easier and 
# nicer to work with.

# I need to do something about the newton's method estimation thing. Cause although distance
# might be close, on the left side of the line filter line moving a tiny distance makes points
# increase exponentially so distance is less of an important measure. Perhaps the distance could
# be dynamic and scale with the left side and right side of the graph. As it approaches the left
# side it needs to be closer and closer?