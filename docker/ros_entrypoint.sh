#!/bin/bash
#set -e

source /opt/ros/$ROS_DISTRO/install/setup.bash

# source workspace setup if available
test -f "/home/developer/driverless_ws/install/setup.bash" && source "/home/developer/driverless_ws/install/setup.bash"

exec "$@"
