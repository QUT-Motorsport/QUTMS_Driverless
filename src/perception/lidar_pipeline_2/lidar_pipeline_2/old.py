import math

# Point cloud source
pc_node = '/velodyne_points'

# Detail of logs
loglevel = 'info'

# Printing logs to terminal
print_logs = False
stdout_handler = None

# Max range of points to process (metres)
LIDAR_RANGE = 20

# Delta angle of segments
DELTA_ALPHA = (2 * math.pi) / 128

# Size of bins
BIN_SIZE = 0.14

# Max angle that will be considered for ground lines
T_M = (2 * math.pi) / 152

# Angle considered to be a small slope
T_M_SMALL = 0

# Max y-intercept for a ground plane line
T_B = 0.1

# Threshold of the Root Mean Square Error of the fit (Recommended: 0.2 - 0.5)
T_RMSE = 0.2

# Determines if regression for ground lines should occur between two
# neighbouring bins when they're described by different lines
REGRESS_BETWEEN_BINS = True

# Maximum distance between point and line to be considered part of ground plane
T_D_GROUND = 0.15 # changed from 0.1

# Maximum distance a point can be from the origin to even be considered as
# a ground point. Otherwise it's labelled as a non-ground point.
T_D_MAX = 100

# Path to data to import and use
data_path = None

# Creates and saves plots
create_figures = False

# Creates, saves and displays plots to the screen
show_figures = False

# Creates animations of figures
animate_figures = False

# Models the car within the figures
model_car = False 

# Export numpy point clouds to text file
export_data = False

import config
general_config = config.general_config()
print(general_config.pc_node)
general_config.pc_node = "test"
print(general_config.pc_node)