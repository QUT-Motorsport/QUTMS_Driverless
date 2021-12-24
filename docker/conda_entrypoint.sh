#!/bin/bash
set -e

source "/home/developer/mambaforge/bin/activate"
conda activate driverless_env
exec "$@"
