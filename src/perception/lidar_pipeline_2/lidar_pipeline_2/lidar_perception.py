# Import ROS2 Modules
import rclpy
from rclpy.node import Node

# Import ROS2 Message Modules
from sensor_msgs.msg import PointCloud2

# Import Custom Message Modules
from driverless_msgs.msg import ConeDetectionStamped

# Import ROS2 Helper Modules
import ros2_numpy as rnp

# Import Custom Modules
from .library import lidar_manager

# Import Python Modules
import numpy as np
import datetime
import pathlib
import os
import getopt
import sys
import time
import math

# Import Logging
import logging
LOGGER = logging.getLogger(__name__)


# Creates timestamp
def create_timestamp():
    return datetime.datetime.now().strftime('%d_%m_%Y_%H_%M_%S_%f')[:-3]


class ConeSensingNode(Node):
    def __init__(self,
                 pc_node,
                 _LIDAR_RANGE,
                 _DELTA_ALPHA,
                 _BIN_SIZE,
                 _T_M,
                 _T_M_SMALL,
                 _T_B,
                 _T_RMSE,
                 _REGRESS_BETWEEN_BINS,
                 _T_D_MAX,
                 _create_figures,
                 _show_figures,
                 _animate_figures,
                 _export_data,
                 _print_logs,
                 _stdout_handler,
                 _working_dir):
        super().__init__('cone_sensing')
        LOGGER.info('Initialising ConeSensingNode')

        self.pc_subscription = self.create_subscription(
            PointCloud2,
            pc_node,
            self.pc_callback,
            2)
        self.pc_subscription  # Prevent unused variable warning

        self.cone_publisher = self.create_publisher(
            ConeDetectionStamped,
            'cone_sensing/cones',
            5)

        self.count = 0

        # Main variables for lidar manager
        self.LIDAR_RANGE = _LIDAR_RANGE
        self.DELTA_ALPHA = _DELTA_ALPHA
        self.BIN_SIZE = _BIN_SIZE
        self.T_M = _T_M
        self.T_M_SMALL = _T_M_SMALL
        self.T_B = _T_B
        self.T_RMSE = _T_RMSE
        self.REGRESS_BETWEEN_BINS = _REGRESS_BETWEEN_BINS
        self.T_D_MAX = _T_D_MAX

        # Misc variables for lidar manager
        self.create_figures = _create_figures
        self.show_figures = _show_figures
        self.animate_figures = _animate_figures
        self.export_data = _export_data
        self.print_logs = _print_logs
        self.stdout_handler = _stdout_handler
        self.working_dir = _working_dir

        LOGGER.info('Waiting for PointCloud2 data ...')

    def pc_callback(self, pc_msg):
        timestamp = create_timestamp()
        LOGGER.info('PointCloud2 message received at ' + timestamp)

        # Convert PointCloud2 message from LiDAR sensor to numpy array
        start_time = time.time()
        dtype_list = rnp.point_cloud2.fields_to_dtype(pc_msg.fields, pc_msg.point_step)  # x y z intensity ring
        point_cloud = np.frombuffer(pc_msg.data, dtype_list)
        end_time = time.time()

        LOGGER.info(f'PointCloud2 converted to numpy array in {end_time - start_time}s')
        LOGGER.debug(point_cloud)

        # Calculating the normal of each point
        start_time = time.time()
        point_norms = np.linalg.norm([point_cloud['x'], point_cloud['y']], axis=0)

        # Creating mask to remove points outside of range and norms of 0
        mask = (point_norms <= self.LIDAR_RANGE) & (point_norms != 0)

        # Applying mask
        point_norms = point_norms[mask]
        point_cloud = point_cloud[mask]
        end_time = time.time()

        LOGGER.info(f'Norm computed and out of range points removed in {end_time - start_time}s')

        # Number of points in point cloud
        point_count = point_cloud.shape[0]
        LOGGER.info(f'POINT_COUNT = {point_count}')
        
        if self.export_data:
            point_clouds_folder = self.working_dir + '/exports'
            if not os.path.isdir(point_clouds_folder):
                os.mkdir(point_clouds_folder)

            point_clouds_folder += "/" + timestamp
            if not os.path.isdir(point_clouds_folder):
                os.mkdir(point_clouds_folder)

            np.savetxt(point_clouds_folder + "/point_cloud.txt", point_cloud)
            np.savetxt(point_clouds_folder + "/point_norms.txt", point_norms)
            print(dtype_list)

        # Identify cones within the received point cloud
        pc_cones = lidar_manager.detect_cones(point_cloud,
                                              point_norms,
                                              self.LIDAR_RANGE,
                                              self.DELTA_ALPHA,
                                              self.BIN_SIZE,
                                              self.T_M,
                                              self.T_M_SMALL,
                                              self.T_B,
                                              self.T_RMSE,
                                              self.REGRESS_BETWEEN_BINS,
                                              self.T_D_MAX,
                                              point_count,
                                              self.create_figures,
                                              self.show_figures,
                                              self.animate_figures,
                                              self.print_logs,
                                              self.stdout_handler,
                                              self.working_dir,
                                              timestamp)

        self.count += 1

        # Publish identified cones
        cones_msg = ConeDetectionStamped(
            header=pc_msg.header,
            cones=pc_cones
        )

        self.cone_publisher.publish(cones_msg)

        total_time = time.time() - start_time
        LOGGER.info(f'Total Time: {total_time}s | Est. Hz: {1 / total_time}')


def main(args=sys.argv[1:]):
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

    # Export numpy point clouds to text file
    export_data = False

    # Processing args
    opts, arg = getopt.getopt(args, str(), ['pc_node=',
                                            'log=',
                                            'lidar_range=',
                                            'delta_alpha=',
                                            'bin_size=',
                                            't_m=',
                                            't_m_small=',
                                            't_b=',
                                            't_rmse=',
                                            't_d_max=',
                                            'import_data=',
                                            'disable_regress',
                                            'create_figures',
                                            'show_figures',
                                            'animate_figures',
                                            'export_data',
                                            'print_logs'])

    for opt, arg in opts:
        if opt == '--pc_node':
            pc_node = arg
        elif opt == '--log':
            loglevel = arg
        elif opt == '--lidar_range':
            LIDAR_RANGE = int(arg)
        elif opt == '--delta_alpha':
            DELTA_ALPHA = arg
        elif opt == '--bin_size':
            BIN_SIZE = arg
        elif opt == '--t_m':
            T_M = arg
        elif opt == '--t_m_small':
            T_M_SMALL = arg
        elif opt == '--t_b':
            T_B = arg
        elif opt == '--t_rmse':
            T_RMSE = arg
        elif opt == '--t_d_max':
            T_D_MAX = arg
        elif opt == '--import_data':
            data_path = "./src/perception/lidar_pipeline_2/lidar_pipeline_2/exports/" + arg
        elif opt == '--disable_regress':
            REGRESS_BETWEEN_BINS = False
        elif opt == '--create_figures':
            create_figures = True
        elif opt == '--show_figures':
            create_figures = True
            show_figures = True
        elif opt == '--animate_figures':
            create_figures = True
            animate_figures = True
        elif opt == '--export_data':
            export_data = True
        elif opt == '--print_logs':
            print_logs = True

    if not print_logs:
        print("--print_logs flag not specified")
        print("Launching lidar_perception without printing to terminal ...")

    # Validating args
    numeric_level = getattr(logging, loglevel.upper(), None)

    if not isinstance(numeric_level, int):
        raise ValueError('Invalid log level: %s' % loglevel)

    # Setting up logging
    working_dir = str(pathlib.Path(__file__).parent.resolve())
    logs_folder = working_dir + '/logs'
    if not os.path.isdir(logs_folder):
        os.mkdir(logs_folder)

    date = datetime.datetime.now().strftime('%d_%m_%Y_%H_%M_%S')
    logging.basicConfig(filename=f'{working_dir}/logs/{date}.log',
                        filemode='w',
                        # Remove levelname s ?
                        format='%(asctime)s | %(levelname)s:%(name)s: %(message)s',
                        datefmt='%I:%M:%S %p',
                        # encoding='utf-8',
                        level=numeric_level)

    # Printing logs to terminal
    if print_logs:
        stdout_handler = logging.StreamHandler(sys.stdout)
        LOGGER.addHandler(stdout_handler)

    LOGGER.info('Hi from lidar_pipeline_2.')
    LOGGER.info(f'args = {args}')

    # Use local data or real-time stream
    if data_path != None:
        point_cloud = np.loadtxt(data_path + "/point_cloud.txt", dtype=np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32), ('ring', np.uint16)]))
        point_norms = np.loadtxt(data_path + "/point_norms.txt")
        point_count = point_cloud.shape[0]
        timestamp = create_timestamp()

        pc_cones = lidar_manager.detect_cones(point_cloud,
                                              point_norms,
                                              LIDAR_RANGE,
                                              DELTA_ALPHA,
                                              BIN_SIZE,
                                              T_M,
                                              T_M_SMALL,
                                              T_B,
                                              T_RMSE,
                                              REGRESS_BETWEEN_BINS,
                                              T_D_MAX,
                                              point_count,
                                              create_figures,
                                              show_figures,
                                              animate_figures,
                                              print_logs,
                                              stdout_handler,
                                              working_dir,
                                              timestamp)
    else:
        # Setting up node
        rclpy.init(args=args)

        cone_sensing_node = ConeSensingNode(pc_node,
                                            LIDAR_RANGE,
                                            DELTA_ALPHA,
                                            BIN_SIZE,
                                            T_M,
                                            T_M_SMALL,
                                            T_B,
                                            T_RMSE,
                                            REGRESS_BETWEEN_BINS,
                                            T_D_MAX,
                                            create_figures,
                                            show_figures,
                                            animate_figures,
                                            export_data,
                                            print_logs,
                                            stdout_handler,
                                            working_dir)

        rclpy.spin(cone_sensing_node)

        # Destroy the node explicitly
        cone_sensing_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])

# Notes
# 1. Intead of rounding the point cloud, see if setting the dtype to
#    16 bit float does the same thing / is faster. Rouding might still
#    leave the numbers as float32s
# 2. Regress between bins is true by default
# 3. Add 'waiting for point cloud after each'
# 4. Add 'have you built cython files?' and wiki
# 5. Unify logging directories
# 6. Fix the total time estimates
# 7. Only import what's required
