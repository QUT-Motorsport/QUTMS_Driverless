import math
import getopt

class general_config:
    def __init__(self):
        self._pc_node = '/velodyne_points'
        self._loglevel = 'info'
        self._print_logs = False
        self._data_path = None
        self._create_figures = False
        self._show_figures = False
        self._animate_figures = False
        self._model_car = False
        self._export_data = False
    
    @property
    def pc_node(self):
        """ 
        Returns:
            str: Point Cloud source
        """
        return self._pc_node
    
    @pc_node.setter
    def pc_node(self, value):
        self._pc_node = value
    
    @property
    def loglevel(self):
        """
        Returns:
            str: Level of detail of logs
        """
        return self._loglevel
    
    @loglevel.setter
    def loglevel(self, value):
        self._loglevel = value
    
    @property
    def print_logs(self):
        """
        Returns:
            bool: Printing logs to terminal
        """
        return self._print_logs
    
    @print_logs.setter
    def print_logs(self, value):
        self._print_logs = value
    
    @property
    def create_figures(self):
        """
        Returns:
            bool: Creates and saves plots
        """
        return self._create_figures
    
    @create_figures.setter
    def create_figures(self, value):
        self._create_figures = value
    
    @property
    def show_figures(self):
        """
        Returns:
            bool: Creates, saves and displays plots to the screen
        """
        return self._show_figures
    
    @show_figures.setter
    def show_figures(self, value):
        self._show_figures = value
    
    @property
    def animate_figures(self):
        """
        Returns:
            bool: Creates animations of figures
        """
        return self._animate_figures
    
    @animate_figures.setter
    def animate_figures(self, value):
        self._animate_figures = value
    
    @property
    def model_car(self):
        """
        Returns:
            bool: Models the car within the figures
        """
        return self._model_car
    
    @model_car.setter
    def model_car(self, value):
        self._model_car = value
    
    @property
    def export_data(self):
        """
        Returns:
            bool: Export numpy point clouds to text file
        """
        return self._export_data
    
    @export_data.setter
    def export_data(self, value):
        self._export_data = value
    
    @property
    def data_path(self):
        """
        Returns:
            str: Path to data to import and use
        """
        return self._data_path
    
    @data_path.setter
    def data_path(self, value):
        self._data_path = value
    

class lidar_config:
    def __init__(self):
        self._LIDAR_RANGE = 20
        self._DELTA_ALPHA = (2 * math.pi) / 128
        self._BIN_SIZE = 0.14
        self._T_M = (2 * math.pi) / 152
        self._T_M_SMALL = 0
        self._T_B = 0.1
        self._T_RMSE = 0.2
        self._REGRESS_BETWEEN_BINS = True
        self._T_D_GROUND = 0.15
        self._T_D_MAX = 100

    @property
    def LIDAR_RANGE(self):
        """ 
        Returns:
            float: Max range of points to process (metres)
        """
        return self._LIDAR_RANGE
    
    @LIDAR_RANGE.setter
    def LIDAR_RANGE(self, value):
        self._LIDAR_RANGE = value
    
    @property
    def DELTA_ALPHA(self):
        """ 
        Returns:
            float: Angle between segments
        """
        return self._DELTA_ALPHA
    
    @DELTA_ALPHA.setter
    def DELTA_ALPHA(self, value):
        self._DELTA_ALPHA = value
    
    @property
    def BIN_SIZE(self):
        """ 
        Returns:
            float: Size of bins
        """
        return self._BIN_SIZE
    
    @BIN_SIZE.setter
    def BIN_SIZE(self, value):
        self._BIN_SIZE = value
    
    @property
    def T_M(self):
        """ 
        Returns:
            float: Max angle that will be considered for ground lines
        """
        return self._T_M
    
    @T_M.setter
    def T_M(self, value):
        self._T_M = value
    
    @property
    def T_M_SMALL(self):
        """ 
        Returns:
            float: Angle considered to be a small slope
        """
        return self._T_M_SMALL
    
    @T_M_SMALL.setter
    def T_M_SMALL(self, value):
        self._T_M_SMALL = value
    
    @property
    def T_B(self):
        """ 
        Returns:
            float: Max y-intercept for a ground plane line
        """
        return self._T_B
    
    @T_B.setter
    def T_B(self, value):
        self._T_B = value

    @property
    def T_RMSE(self):
        """ 
        Returns:
            float: Threshold of the Root Mean Square Error of the fit (Recommended: 0.2 - 0.5)
        """
        return self._T_RMSE
    
    @T_RMSE.setter
    def T_RMSE(self, value):
        self._T_RMSE = value
    
    @property
    def REGRESS_BETWEEN_BINS(self):
        """ 
        Returns:
            float: Determines if regression for ground lines should occur between two neighbouring bins when they're described by different lines
        """
        return self._REGRESS_BETWEEN_BINS
    
    @REGRESS_BETWEEN_BINS.setter
    def REGRESS_BETWEEN_BINS(self, value):
        self._REGRESS_BETWEEN_BINS = value
    
    @property
    def T_D_GROUND(self):
        """ 
        Returns:
            float: Maximum distance between point and line to be considered part of ground plane # changed from 0.1
        """
        return self._T_D_GROUND
    
    @T_D_GROUND.setter
    def T_D_GROUND(self, value):
        self._T_D_GROUND = value
    
    @property
    def T_D_MAX(self):
        """ 
        Returns:
            float: Maximum distance a point can be from the origin to even be considered as a ground point. Otherwise it's labelled as a non-ground point
        """
        return self._T_D_MAX
    
    @T_D_MAX.setter
    def T_D_MAX(self, value):
        self._T_D_MAX = value

def process_args(args, general_config, lidar_config):
    opts, arg = getopt.getopt(args, str(), ['pc_node=',
                                            'loglevel=',
                                            'lidar_range=',
                                            'delta_alpha=',
                                            'bin_size=',
                                            't_m=',
                                            't_m_small=',
                                            't_b=',
                                            't_rmse=',
                                            't_d_ground=',
                                            't_d_max=',
                                            'import_data=',
                                            'disable_regress',
                                            'create_figures',
                                            'show_figures',
                                            'animate_figures',
                                            'model_car',
                                            'export_data',
                                            'print_logs'])
    
    for opt, arg in opts:
        if opt == '--pc_node':
            pc_node = arg
        elif opt == '--loglevel':
            loglevel = arg
        elif opt == '--lidar_range':
            LIDAR_RANGE = float(arg)
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
        elif opt == '--t_d_ground':
            T_D_GROUND = float(arg)
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
        elif opt == '--model_car':
            create_figures = True
            model_car = True
        elif opt == '--export_data':
            export_data = True
        elif opt == '--print_logs':
            print_logs = True