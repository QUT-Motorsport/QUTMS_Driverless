#!/bin/bash

# apparently `set -e` ("strict mode") breaks conda activate, see:
# https://pythonspeed.com/articles/activate-conda-dockerfile/#another-working-solution

source "/home/developer/mambaforge/bin/activate"
conda activate driverless_env

# source workspace setup if available
test -f "/home/developer/driverless_ws/install/setup.bash" && source "/home/developer/driverless_ws/install/setup.bash"

exec "$@"
