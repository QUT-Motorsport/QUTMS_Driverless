import pathlib

# Config Parameters
WORKING_DIR: str = str(pathlib.Path(__file__).parent.resolve())
OUTPUT_DIR: str = WORKING_DIR + '/output'

# Algorithm Parameters
LIDAR_RANGE = 100            # Max range of points to process (metres)