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

    # General
    loglevel = 'info' # 
    print_logs = False # Printing logs to terminal
    data_path = None # Path to data to import and use
    create_figures = False # Creates and saves plots
    show_figures = False # Creates, saves and displays plots to the screen    
    animate_figures = False # Creates animations of figures
    model_car = False # Models the car within the figures
    export_data = False # Export numpy point clouds to text file
    
    @property
    def pc_node(self):
        """ Point Cloud source

        Returns:
            _type_: _description_
        """
        return self._pc_node
    
    @pc_node.setter
    def pc_node(self, value):
        self.pc_node = value
    
    @property
    def loglevel(self):
        """ Level of detail of logs

        Returns:
            _type_: _description_
        """
        return self._loglevel
    
    @pc_node.setter
    def loglevel(self, value):
        self._loglevel = value
    
    @property
    def pc_node(self):
        """ Point Cloud source

        Returns:
            _type_: _description_
        """
        return self.pc_node
    
    @pc_node.setter
    def pc_node(self, value):
        self.pc_node = value
    
    @property
    def pc_node(self):
        """ Point Cloud source

        Returns:
            _type_: _description_
        """
        return self.pc_node
    
    @pc_node.setter
    def pc_node(self, value):
        self.pc_node = value
    
    @property
    def pc_node(self):
        """ Point Cloud source

        Returns:
            _type_: _description_
        """
        return self.pc_node
    
    @pc_node.setter
    def pc_node(self, value):
        self.pc_node = value
    
    @property
    def pc_node(self):
        """ Point Cloud source

        Returns:
            _type_: _description_
        """
        return self.pc_node
    
    @pc_node.setter
    def pc_node(self, value):
        self.pc_node = value
    
    @property
    def pc_node(self):
        """ Point Cloud source

        Returns:
            _type_: _description_
        """
        return self.pc_node
    
    @pc_node.setter
    def pc_node(self, value):
        self.pc_node = value
    
    @property
    def pc_node(self):
        """ Point Cloud source

        Returns:
            _type_: _description_
        """
        return self.pc_node
    
    @pc_node.setter
    def pc_node(self, value):
        self.pc_node = value
    
    @property
    def pc_node(self):
        """ Point Cloud source

        Returns:
            _type_: _description_
        """
        return self.pc_node
    
    @pc_node.setter
    def pc_node(self, value):
        self.pc_node = value
    
    

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
