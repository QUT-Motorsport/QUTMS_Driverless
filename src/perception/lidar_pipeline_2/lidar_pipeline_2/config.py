import math

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
        self.pc_node = value
    
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
    # Algorithm
    LIDAR_RANGE = 20 # Max range of points to process (metres)
    DELTA_ALPHA = (2 * math.pi) / 128 # Angle between segments
    BIN_SIZE = 0.14 # Size of bins
    T_M = (2 * math.pi) / 152 # Max angle that will be considered for ground lines
    T_M_SMALL = 0 # Angle considered to be a small slope
    T_B = 0.1 # Max y-intercept for a ground plane line
    T_RMSE = 0.2 # Threshold of the Root Mean Square Error of the fit (Recommended: 0.2 - 0.5)
    REGRESS_BETWEEN_BINS = True # Determines if regression for ground lines should occur between two neighbouring bins when they're described by different lines
    T_D_GROUND = 0.15 # Maximum distance between point and line to be considered part of ground plane # changed from 0.1
    T_D_MAX = 100 # Maximum distance a point can be from the origin to even be considered as a ground point. Otherwise it's labelled as a non-ground point.
