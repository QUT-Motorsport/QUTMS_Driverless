#!/bin/bash

echo ""
echo "---Creating driverless env---"
echo ""
sleep 3
cd ~/QUTMS/QUTMS_Driverless
mamba env create --name driverless_env --file installation/humble_py310_dev_env.yml
conda activate driverless_env
conda config --env --add channels conda-forge
conda config --env --add channels robostack-staging
mamba install -y --file installation/conda_requirements.txt
