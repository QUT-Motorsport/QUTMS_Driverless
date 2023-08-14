geometry_msgs/Point location
uint8 color # use the enum below

# TESTING ONLY - used for known data association from the sim
int8 sim_cone_index  # default value of 0 means unused or unknown

# Constants
uint8 BLUE=0
uint8 UNKNOWN=1
uint8 ORANGE_BIG=2
uint8 ORANGE_SMALL=3
uint8 YELLOW=4
