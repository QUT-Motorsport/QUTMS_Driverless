#!/bin/bash

cd ~

## Pre-reqs
echo ""
echo "---Installing pre-reqs---"
echo ""
sleep 3
sudo apt update
sudo apt upgrade -y
sudo apt install -y git python3-pip pre-commit cmake

## Set ROS2 repositories
source ~/QUTMS/QUTMS_Driverless/installation/setup_scripts/install_ros_source.sh

## Install ROS2 Humble
source ~/QUTMS/QUTMS_Driverless/installation/setup_scripts/install_ros_humble.sh

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
cd QUTMS_Driverless

## Package requirements
echo ""
echo "---Installing driverless package requirements---"
echo ""
sleep 3
pip install -r installation/requirements.txt

## Create an alias for ease
echo "alias a='source install/setup.bash'" >> ~/.bashrc

## Check if user is going to work with vision
source ~/QUTMS/QUTMS_Driverless/installation/setup_scripts/install_torch_cuda.sh

## Source ROS
source /opt/ros/humble/setup.bash

## Install EUFS
source ~/QUTMS/QUTMS_Driverless/installation/setup_scripts/install_eufs_sim.sh

## Install dependencies from src/
rosdep install --from-paths ~/QUTMS --ignore-src -r -y

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
