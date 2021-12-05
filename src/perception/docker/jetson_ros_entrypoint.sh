#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/foxy/install/setup.bash"
source "/home/developer/driverless_ws/install/setup.bash"
exec "$@"
