#!/bin/bash

cd ~

## Pre-reqs
echo ""
echo "---Installing pre-reqs---"
echo ""
sleep 3
sudo apt update
sudo apt upgrade -y
sudo apt install -y git python3-pip pre-commit mesa-utils

## Make workspace
mkdir ~/QUTMS
cd ~/QUTMS
# directory for people to drop their rosbags in
mkdir bags/

## Clone Driverless repo
echo ""
echo "---Cloning Driverless repo---"
echo ""
sleep 3
git clone --recurse-submodules https://github.com/QUT-Motorsport/QUTMS_Driverless.git

## Download and install mambaforge
source ~/QUTMS/QUTMS_Driverless/installation/setup_scripts/install_mambaforge.sh

## Create driverless development environment
source ~/QUTMS/QUTMS_Driverless/installation/setup_scripts/install_conda_env.sh

## Create an alias for ease
echo "alias a='conda activate driverless_env && source install/setup.bash'" >> ~/.bashrc

## Check if user is going to work with vision
source ~/QUTMS/QUTMS_Driverless/installation/setup_scripts/install_torch_cuda.sh

## Install EUFS
source ~/QUTMS/QUTMS_Driverless/installation/setup_scripts/install_eufs_sim.sh

# echo ""
# echo "---Building packages---"
# echo ""
# sleep 3
# colcon build --symlink-install

## Pre commit for git
source ~/QUTMS/QUTMS_Driverless/installation/setup_scripts/install_pre-commit.sh

## Check if user wants to install WSL version of GitKraken
source ~/QUTMS/QUTMS_Driverless/installation/install_gitkraken.sh

## Wrap up
echo "Thank you for installing, explore the rest of the wiki"
