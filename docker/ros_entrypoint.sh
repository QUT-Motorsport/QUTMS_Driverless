#!/bin/bash
#set -e

echo 'export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"' >> /home/developer/.bashrc
echo 'export CYCLONEDDS_URI=file:///home/developer/driverless_ws/cyclonedds.xml' >> /home/developer/.bashrc
echo 'export ROS_DOMAIN_ID=69' >> /home/developer/.bashrc

export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
export CYCLONEDDS_URI="file:///home/developer/driverless_ws/cyclonedds.xml"
export ROS_DOMAIN_ID=69

test -f "/opt/ros/$ROS_DISTRO/setup.bash" && source "/opt/ros/$ROS_DISTRO/setup.bash"
test -f "/opt/ros/$ROS_DISTRO/install/setup.bash" && source "/opt/ros/$ROS_DISTRO/install/setup.bash"

# source workspace setup if available
test -f "/home/developer/driverless_ws/install/setup.bash" && source "/home/developer/driverless_ws/install/setup.bash"

exec "$@"
