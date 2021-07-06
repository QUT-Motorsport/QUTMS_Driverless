import math

# INCORRECT POINT WISE VARIANCE. HACKY FIX:
# Point Wise Variance 
def get_point_var(x, y, x_mean, y_mean):
    x_var = (x - x_mean)**2
    y_var = (y - y_mean)**2
    return (x_var + y_var) / 2

# Sample Variances
def get_sample_vars(points, x_mean, y_mean, num_points):
    x_sample_var = 0
    y_sample_var = 0
    for i in range(num_points):
        x_sample_var += (points[i][0] - x_mean)**2
        y_sample_var += (points[i][1] - y_mean)**2
    x_sample_var = x_sample_var / (num_points - 1)
    y_sample_var = y_sample_var / (num_points - 1)
    return [x_sample_var, y_sample_var]

# Theta
def get_theta(x, y):
    return math.atan(y/x)

# Theta Vector
def get_theta_vector(points, num_points):
    theta_mat = []
    for i in range(num_points):
        theta_mat.append(get_theta(points[i][0], points[i][1]))
    return theta_mat

# Rho
def get_rho(x, y):
    return math.sqrt(x**2 + y**2)

# Rho Vector
def get_rho_vector(points, num_points):
    rho_mat = []
    for i in range(num_points):
        rho_mat.append(get_rho(points[i][0], points[i][1]))
    return rho_mat

# Weights, W
def get_weights(points, num_points, x_mean, y_mean):
    w_mat = []
    for i in range(num_points):
        w_mat.append(get_point_var(points[i][0], points[i][1], x_mean, y_mean))
    return w_mat

# x_w
def get_x_weight(w_mat, rho_mat, theta_mat, w_mean, num_points):
    x_weight = 0
    for i in range(num_points):
        x_weight += w_mat[i] * rho_mat[i] * math.cos(theta_mat[i])
    return (1/w_mean) * x_weight

# y_w
def get_y_weight(w_mat, rho_mat, theta_mat, w_mean, num_points):
    y_weight = 0
    for i in range(num_points):
        y_weight += w_mat[i] * rho_mat[i] * math.sin(theta_mat[i])
    return (1/w_mean) * y_weight

# alpha, rearrange (4) to get alpha
def get_alpha(points, num_points, w_mat, x_weight, y_weight):
    numerator = 0
    denominator = 0
    for i in range(num_points):
        numerator += w_mat[i] * (y_weight - points[i][1]) * (x_weight - points[i][0])
        denominator += w_mat[i] * ((y_weight - points[i][1])**2 - (x_weight - points[i][0])**2)
    return math.tan((-2 * numerator) / denominator)

# r
def get_r(x_weight, y_weight, alpha):
    return x_weight * math.cos(alpha) + y_weight * math.sin(alpha)
