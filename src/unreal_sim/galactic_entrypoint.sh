#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/galactic/setup.bash"

# source workspace setup if available
test -f "/home/developer/driverless_ws/install/setup.bash" && source "/home/developer/driverless_ws/install/setup.bash"

exec "$@"
