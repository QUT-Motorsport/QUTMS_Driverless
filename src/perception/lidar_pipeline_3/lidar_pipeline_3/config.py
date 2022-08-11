import datetime
import getopt
import logging
import os

from . import constants as const

from typing import Any, Tuple

class Config:
    def __init__(self) -> None:
        # Parameters
        self._pc_node: str = '/velodyne_points'
        self._loglevel: str = 'info'
        self._print_logs: bool = False
        self._data_path: str = ''
        self._create_figures: bool = False
        self._show_figures: bool = False
        self._animate_figures: bool = False
        self._plot_car: bool = False
        self._export_data: bool = False
        
        # Misc
        self._timestamp: str
        self._datestamp: str
        self._datetimestamp: str
        self._timestamp, self._datestamp, self._datetimestamp = self.get_timestamp()
        self._runtime_dir = const.OUTPUT_DIR + '/' + self.datetimestamp
        
        self.setup_output_dir()
        self.setup_runtime_dir()
    
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
        numeric_level: Any = getattr(logging, value.upper(), None)
        if not isinstance(numeric_level, int):
            raise ValueError(f'Invalid log level: {value}')
        
        self._loglevel = value
    
    @property
    def numeric_loglevel(self) -> Any:
        """
        Returns:
            str: Return numeric (int) level of detail of logs
        """
        return getattr(logging, self.loglevel.upper(), None)
    
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
    def plot_car(self) -> bool:
        """
        Returns:
            bool: Plots the car within the figures
        """
        return self._plot_car
    
    @plot_car.setter
    def plot_car(self, value) -> None:
        self._plot_car = value
    
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
    
    @property
    def timestamp(self) -> str:
        """
        Returns:
            str: Time at init
        """
        return self._timestamp
    
    @property
    def datestamp(self) -> str:
        """
        Returns:
            str: Date at init
        """
        return self._datestamp
    
    @property
    def datetimestamp(self) -> str:
        """
        Returns:
            str: Date and time at init
        """
        return self._datetimestamp
    
    @property
    def runtime_dir(self) -> str:
        """
        Returns:
            str: Runtime directory
        """
        return self._runtime_dir
    
    def setup_output_dir(self):
        """Create output directory if it does not exist
        """
        if not os.path.isdir(const.OUTPUT_DIR):
            os.mkdir(const.OUTPUT_DIR)
    
    def setup_runtime_dir(self):
        """Create runtime directory if it does not exist
        """
        if not os.path.isdir(self.runtime_dir):
            os.mkdir(self.runtime_dir)
    
    def update(self, args: list) -> None:
        """Update config values with user input

        Args:
            args (list): List of flags provided by the user from the command line
        """
        opts, arg = getopt.getopt(args, '', ['pc_node=',
                                            'loglevel=',
                                            'data_path=',
                                            'create_figures',
                                            'show_figures',
                                            'animate_figures',
                                            'plot_car',
                                            'export_data',
                                            'print_logs'])
        
        for opt, arg in opts:
            if opt == '--pc_node':
                self.pc_node = arg
            elif opt == '--loglevel':
                self.loglevel = arg
            elif opt == '--data_path':
                self.data_path = arg
            elif opt == '--create_figures':
                self.create_figures = True
            elif opt == '--show_figures':
                self.show_figures = True
            elif opt == '--animate_figures':
                self.animate_figures = True
            elif opt == '--plot_car':
                self.plot_car = True
            elif opt == '--export_data':
                self.export_data = True
            elif opt == '--print_logs':
                self.print_logs = True
    
    def get_timestamp(self) -> Tuple[str, str, str]:
        datetimestamp: str = datetime.datetime.now().strftime('%d_%m_%Y_%H_%M_%S_%f')[:-3]
        timestamp: str = datetimestamp[:10]
        datestamp: str = datetimestamp[11:]
        return (timestamp, datestamp, datetimestamp)
