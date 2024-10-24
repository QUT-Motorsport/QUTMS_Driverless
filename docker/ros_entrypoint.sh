#!/bin/bash
#set -e

test -f "/opt/ros/$ROS_DISTRO/setup.bash" && source "/opt/ros/$ROS_DISTRO/setup.bash"
test -f "/opt/ros/$ROS_DISTRO/install/setup.bash" && source "/opt/ros/$ROS_DISTRO/install/setup.bash"

# source workspace setup if available
test -f "/home/developer/driverless_ws/install/setup.bash" && source "/home/developer/driverless_ws/install/setup.bash"

exec "$@"
