import math
import copy
import total_least_squares

T_M = 2*math.pi / 128 # Max angle that will be considered as ground plane # 45 deg
T_M_SMALL = math.pi / 128  # Angle considered to be a small slope # 5.625 deg
T_B = 0.1 # Max y-intercept for a ground plane line # 5 cm 
T_RMSE = 0.2 # Threshold of the Root Mean Square Error of the fit # Recommended: 0.2 - 0.5
T_D_PREV = 0.5 # Max distance of the first point of a line to the line previously fitted # 1 m
# Unsure of how T_D_PREV impacts line_extraction

# Returns the distance from a point to a line
def dist_point_line_old(point, m_c, b_c):
    x_p = point[0]
    y_p = point[1]
    m_p = (-1) / m_c # gradient perpendicular line
    b_p = -(y_p - m_p) * x_p # intercept of perpendicular line
    x_shared = (b_p - b_c) / (m_c - m_p)
    y_shared = m_p*x_p + b_p # or m_c*x + b_c # at x_shared, y_p = y_c
    #print(math.sqrt((x_shared + x_p)**2 + y_shared**2))
    return math.sqrt((x_shared + x_p)**2 + y_shared**2)

# https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
def dist_point_line_old_now_too(point, m_c, b_c):
    x_0 = point[0]
    y_0 = point[1]
    a_x = m_c * x_0
    b_y = -1 * y_0

    numer = abs(a_x + b_y + b_c)
    denom = math.sqrt(a_x**2) # + b**2 cancels out since b is 1
    distance = numer / denom
    if distance > 0.5:
        print(x_0, y_0, m_c, b_c, distance)
    return distance

# https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
def dist_point_line_revised(point, m_c, b_c):
    x_0 = point[0]
    y_0 = point[1]
    a = m_c
    b = 1
    c = b_c
    distance = abs(a*x_0 + b*y_0 + c) / math.sqrt(a**2 + b**2)
    if distance > 0.5:
        print(x_0, y_0, m_c, b_c, distance)
    return distance

# https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
def dist_point_line(point, m_c, b_c):
    x_0 = point[0]
    y_0 = point[1]
    distance = abs(m_c*x_0 + y_0 + b_c) / math.sqrt(m_c**2 + 1)
    return distance

# Returns the Root Mean Square Error (RMSE) of points about a regression line
def fit_error_old(m, b, points):
    num_points = len(points)
    x_mean = sum([point[0] for point in points]) / num_points
    y_mean = sum([point[1] for point in points]) / num_points

    ssr = 0 # Sums of squares due to regression 
    sse = 0 # Sums of squares error
    sd_y = 0 # sample standard deviation of y
    for i in range(num_points):
        ssr += (points[i][1] - y_mean)**2
        sse += ((m*points[i][0] + b) - points[i][1])**2
        sd_y += (points[i][0] - x_mean)**2
    sst = ssr + sse
    r2 = ssr / sst
    sd_y = math.sqrt(sd_y / (num_points - 1))

    return math.sqrt(1 - r2) * sd_y

def fit_error(m, b, points):
    num_points = len(points)
    sse = 0
    
    for i in range(num_points):
        x = points[i][0]
        best_fit = m*x + b
        observed = points[i][1]
        sse += (best_fit - observed)**2
        
    sse_mean = sse / num_points
    rmse = math.sqrt(sse_mean)

    return rmse

# The Incremental Algorithm
def extract_segment_lines(segment, num_bins):
    lines = []
    new_line_points = []
    lines_created = 0
    m_new = None
    b_new = None

    i = 0

    while i < num_bins:
        if len(segment[i]) == 2:
            m_new = None
            b_new = None

            new_point = segment[i]
            if len(new_line_points) >= 2:
                temp_line_points = copy.deepcopy(new_line_points)
                temp_line_points.append(new_point)
                [m_new, b_new] = total_least_squares.fit_line(temp_line_points)
                #print(abs(m_new), m_new, abs(b_new), fit_error(m_new, b_new, temp_line_points))
                if (abs(m_new) <= T_M and (m_new > T_M_SMALL or abs(b_new) <= T_B) and fit_error(m_new, b_new, temp_line_points) <= T_RMSE):
                    new_line_points.append(new_point)
                    temp_line_points = []
                else:
                    # It appears that this if statement might occur for the same point that the 
                    # else statement print("no", new_point, i) does too
                    [m_new, b_new] = total_least_squares.fit_line(new_line_points)
                    #print("Adding line to segment")
                    lines.append([m_new, b_new, new_line_points[0], new_line_points[len(new_line_points) - 1], len(new_line_points)])
                    new_line_points = []
                    lines_created += 1
                    i = i - 1
            else:
                if lines_created == 0 or len(new_line_points) != 0 or dist_point_line(new_point, lines[lines_created-2][0], lines[lines_created-2][1]) <= T_D_PREV:
                    new_line_points.append(new_point)
                else:
                    #print("no", new_point, i)
                    pass
        elif len(segment[i]) > 2 or len(segment[i]) == 1:
            raise ValueError("More than one prototype point has been found in a bin!", "i:", i, "len:", len(segment[i]), "segment[i]:", segment[i])
        #print("i go up", i)
        i += 1
    if len(new_line_points) > 1 and m_new != None and b_new != None:
        lines.append([m_new, b_new, new_line_points[0], new_line_points[len(new_line_points) - 1], len(new_line_points)])
    if (m_new == None and b_new != None) or (m_new != None and b_new == None):
        raise ValueError("how the hell did this happen. Like literally how. it wont, this if statement is unnecessary.")
    return lines

# Returns the ground lines for all segments
def extract_lines(segments_bins_prototype, num_segments, num_bins):
    ground_plane = []
    for i in range(num_segments):
        #print("Extracting lines from Segment:", i+1)
        ground_plane.append(extract_segment_lines(segments_bins_prototype[i], num_bins))
    return ground_plane

# Notes
#   - Constant T_M_SMALL and T_B: For small slopes m < T_M_SMALL, 
#     the line’s absolute y-intercept b must not exceed a certain 
#     threshold T_B. This way, plateaus are excluded from the 
#     ground plane. 
#   - Constant T_RMSE: The root mean square error of the fit must 
#     not exceed a certain threshold T_RMSE. I'm unsure what this
#     value should be yet.
#   - Constant T_D_PREV: The distance of the first point of a line
#     to the line previously fitted must not exceed T_D_PREV, 
#     enforcing smooth transitions between pairs of successive 
#     lines. 
#   - extract_segment_lines(): Only one prototype point is expected 
#     since each bin is reduced to one point with the lowest z value.
#   - IMPORTANT: Isn't it possible for one segment to have multiple lines?
#     If so, this algorithm does not return the rho (norm) of the line,
#     nor does it indicate which bin the line starts and stops in.
#     Currently the line norm is set to the max lidar range and one line
#     is assumed per segment, starting from the origin. 
#   - Alright so I haven't techincally finsihed this. In the visualisation's
#     you can see how the length of each line is simply set as LIDAR_RANGE
#     and I'm not actually returning the real line length of rho from TLS.
#     Additionally, there can be multiple lines in each segment but I'm
#     drawing all simply from the origin rather than starting and ending in
#     the correct bins. Why am I ignoring this for now? Well, I think I can 
#     proceed for cone estimation anyways since all I need to know is what 
#     points are NOT on the ground, which I've achieved.    IMPORTANT