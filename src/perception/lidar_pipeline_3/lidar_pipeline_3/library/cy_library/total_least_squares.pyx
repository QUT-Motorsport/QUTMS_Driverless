# Modules
cimport libc.math as math
import sys


# Point Wise Variance
cdef float get_point_var(int x, int y, float x_mean, float y_mean):
    # point_x_var = (x - x_mean)**2
    # point_y_var = (y - y_mean)**2
    # return (point_x_var + point_y_var) / 2
    return 1.0


# Unused
# Sample Variances
cdef get_sample_vars(points, float x_mean, float y_mean, int num_points):
    cdef float x_sample_var = 0
    cdef float y_sample_var = 0
    cdef int i

    for i in range(num_points):
        x_sample_var += (points[i][0] - x_mean)**2
        y_sample_var += (points[i][1] - y_mean)**2

    x_sample_var = x_sample_var / (num_points - 1)
    y_sample_var = y_sample_var / (num_points - 1)

    return [x_sample_var, y_sample_var]


# Theta
cdef float get_theta(float x, float y):
    if x == 0:
        x = sys.maxsize

    cdef float theta = math.atan(y / x)
    return theta


# Theta Vector
cdef get_theta_vector(points, int num_points):
    theta_mat = []
    cdef int i

    for i in range(num_points):
        theta_mat.append(get_theta(points[i][0], points[i][1]))

    return theta_mat


# Rho
cdef get_rho(float x, float y):
    rho = math.sqrt(x**2 + y**2)
    return rho


# Rho Vector
cdef get_rho_vector(points, int num_points):
    rho_mat = []
    cdef int i

    for i in range(num_points):
        rho_mat.append(get_rho(points[i][0], points[i][1]))

    return rho_mat


# Weights, W
cdef get_weights(points, int num_points, float x_mean, float y_mean):
    w_mat = []
    cdef int i

    for i in range(num_points):
        w_mat.append(get_point_var(points[i][0], points[i][1], x_mean, y_mean))

    return w_mat


# x_w
cdef float get_x_weight(w_mat, rho_mat, theta_mat, w_sum, int num_points):
    cdef float x_weight = 0
    cdef int i

    for i in range(num_points):
        x_weight += w_mat[i] * rho_mat[i] * math.cos(theta_mat[i])

    x_weight = (1/w_sum) * x_weight
    return x_weight


# y_w
cdef float get_y_weight(w_mat, rho_mat, theta_mat, w_sum, int num_points):
    cdef float y_weight = 0
    cdef int i

    for i in range(num_points):
        y_weight += w_mat[i] * rho_mat[i] * math.sin(theta_mat[i])

    y_weight = (1/w_sum) * y_weight
    return y_weight


# alpha, rearrange (4) to get alpha
cdef float get_alpha(points, int num_points, w_mat, float x_weight, float y_weight):
    cdef float numerator = 0
    cdef float denominator = 0
    cdef int i

    for i in range(num_points):
        numerator += w_mat[i] * (y_weight - points[i][1]) * (x_weight - points[i][0])
        denominator += w_mat[i] * ((y_weight - points[i][1])**2 - (x_weight - points[i][0])**2)

    numerator = -2 * numerator
    cdef float tan_2_alpha = numerator / denominator

    # - Custom Code -
    # If the difference in height between first and last point is negative,
    # then make tan_2_alpha negative. Otherwise, positive.
    cdef float y_delta = points[0][1] - points[num_points - 1][1]
    if y_delta >= 0:
        tan_2_alpha = -abs(tan_2_alpha)
    else:
        tan_2_alpha = abs(tan_2_alpha)

    cdef float alpha = math.atan(tan_2_alpha) / 2
    return alpha


# r
cdef float get_r(float x_weight, float y_weight, float alpha):
    cdef float r = x_weight * math.cos(alpha) + y_weight * math.sin(alpha)
    return r


# Gradient, m
cdef float get_m(float alpha, float y_weight):
    cdef float m = math.tan(alpha)
    return m


# Intercept, b
cdef get_b(points, int num_points, float r, float alpha, float m):
    cdef float b = 0
    cdef int i

    for i in range(num_points):
        b += points[i][1] - m * points[i][0]

    b = b / num_points
    return b


# Total Least Squares Fitting Method
cpdef fit_line(points):
    cdef int num_points = len(points)

    cdef float x_mean = sum([point[0] for point in points]) / num_points
    cdef float y_mean = sum([point[1] for point in points]) / num_points

    rho_mat = get_rho_vector(points, num_points)
    theta_mat = get_theta_vector(points, num_points)

    w_mat = get_weights(points, num_points, x_mean, y_mean)
    w_sum = sum(w_mat)

    cdef float x_weight = get_x_weight(w_mat, rho_mat, theta_mat, w_sum, num_points)
    cdef float y_weight = get_y_weight(w_mat, rho_mat, theta_mat, w_sum, num_points)

    cdef float alpha = get_alpha(points, num_points, w_mat, x_weight, y_weight)
    cdef float r = get_r(x_weight, y_weight, alpha)

    if r < 0:
        r = -r
        alpha += math.pi/2

    cdef float m = get_m(alpha, y_weight)
    cdef float b = get_b(points, num_points, r, alpha, m)

    return ([m, b])

# Notes
# 1. The get_point_var() is incorrectly implemented for the time being as I was unable to
#    understand how to calculate point-wise variance. In the paper, it is made clear that
#    each point has a weight and that "The weights can be chosen as the inverses of the
#    variances giving most weight to data points with low variability". The old
#    implementation of get_point_var() finds the variance of the x value and the variance
#    of the y value. It returns the average of these two variances. I don't believe this
#    worked correctly. The current implementation simply returns 1 (i.e. point wise
#    variance is not computed).
# 2. Currently get_sample_vars() is unused since only the variance of each point is neeeded
#    and I'm unsure if the variance of a point is related to the sample variances.
# 3. Investigate faster way to calculate b (intercept), perhaps using the gradient, rho and
#    other values calcualted and relating to polar coordinates. I may have already fixed
#    this. I'm going through today and tidying notes which may or may not be out of date,
#    whoops.
# 4. tan_2_alpha = numerator / denominator will encounter a division by zero error if the
#    points forming the line are EXACTLY linear. However, this is practically impossible.
# 5. get_theta(x, y) attempts to solve a division by zero error by setting x to be the max
#    system value if it is equal to 0. This patch did not work for tan_2_alpha.
# 6. Investigate all places that involve division and explore edge cases that may lead to
#    division by zero errors (like the above).
# 7. Review method for identifying sign of gradient. If first or last point is an outlier, the
#    current method will be unreliable. Perhaps use the average change in height between the points.
#    If the average change is negative, then the gradient should be made negative and vice versa.
#    Additionally, the average change could also exclude one point at a time and observer how much
#    the average changes (to identify and remove outliers). However, this is likely unecessary and
#    too computationally expensive (especially when our goal is near real-time performance). Pretty
#    sure this comment has been fixed as I have gotten the algorithm correctly working.
# 8. Look at any places that use len(x) and see if this can be turned into a parsed constant that
#    doesn't need to be repeatedly re-calculated.

# Sources
#   - Paper: Feature Extraction and Scene Interpretation for Map-Based Navigation and Map Building
#   - Paper: Range Finding and Feature Extraction by Segmentation of Images for Mobile Robot Navigation
#   - https://socratic.org/questions/5a1e20e711ef6b70b8538562
#     IMPORTANT: The site above appears to use rho / p where the papers instead use r, and vice versa
