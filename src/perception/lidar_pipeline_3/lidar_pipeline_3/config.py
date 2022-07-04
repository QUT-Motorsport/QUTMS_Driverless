class Config:
    def __init__(self) -> None:
        self._pc_node: str = '/velodyne_points'
        self._loglevel: str = 'info'
        self._print_logs: bool = False
        self._data_path: str = ''
        self._create_figures: bool = False
        self._show_figures: bool = False
        self._animate_figures: bool = False
        self._model_car: bool = False
        self._export_data: bool = False
    
    @property
    def pc_node(self) -> str:
        """ 
        Returns:
            str: Point Cloud source
        """
        return self._pc_node
    
    @pc_node.setter
    def pc_node(self, value: str) -> None:
        self._pc_node = value
    
    @property
    def loglevel(self) -> str:
        """
        Returns:
            str: Level of detail of logs
        """
        return self._loglevel
    
    @loglevel.setter
    def loglevel(self, value) -> None:
        self._loglevel = value
    
    @property
    def print_logs(self) -> bool:
        """
        Returns:
            bool: Printing logs to terminal
        """
        return self._print_logs
    
    @print_logs.setter
    def print_logs(self, value) -> None:
        self._print_logs = value
    
    @property
    def create_figures(self) -> bool:
        """
        Returns:
            bool: Creates and saves plots
        """
        return self._create_figures
    
    @create_figures.setter
    def create_figures(self, value) -> None:
        self._create_figures = value
    
    @property
    def show_figures(self) -> bool:
        """
        Returns:
            bool: Creates, saves and displays plots to the screen
        """
        return self._show_figures
    
    @show_figures.setter
    def show_figures(self, value) -> None:
        self._show_figures = value
    
    @property
    def animate_figures(self) -> bool:
        """
        Returns:
            bool: Creates animations of figures
        """
        return self._animate_figures
    
    @animate_figures.setter
    def animate_figures(self, value) -> None:
        self._animate_figures = value
    
    @property
    def model_car(self) -> bool:
        """
        Returns:
            bool: Models the car within the figures
        """
        return self._model_car
    
    @model_car.setter
    def model_car(self, value) -> None:
        self._model_car = value
    
    @property
    def export_data(self) -> bool:
        """
        Returns:
            bool: Export numpy point clouds to text file
        """
        return self._export_data
    
    @export_data.setter
    def export_data(self, value) -> None:
        self._export_data = value
    
    @property
    def data_path(self) -> str:
        """
        Returns:
            str: Path to data to import and use
        """
        return self._data_path
    
    @data_path.setter
    def data_path(self, value) -> None:
        self._data_path = value
