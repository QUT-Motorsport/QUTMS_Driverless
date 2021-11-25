#!/bin/bash
set -e

# setup ros2 environment
source "/ros_zed_ws/install/setup.bash"
exec "$@"
