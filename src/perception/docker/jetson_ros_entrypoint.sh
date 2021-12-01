#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/foxy/install/setup.bash"
exec "$@"
