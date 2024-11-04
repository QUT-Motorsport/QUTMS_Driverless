#!/bin/bash
#set -e

test -f "/opt/ros/${ROS_DISTRO}/setup.bash" && source "/opt/ros/$ROS_DISTRO/setup.bash"
echo "Sourced ROS 2 ${ROS_DISTRO}"

# source workspace setup if available
test -f "/home/${USER}/QUTMS/install/setup.bash" && source "/home/$USER/QUTMS/install/setup.bash"
echo "Sourced QUTMS workspace"

exec "$@"
