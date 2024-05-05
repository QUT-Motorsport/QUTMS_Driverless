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

## Set ROS2 repositories
source ~/QUTMS/QUTMS_Driverless/installation/install_scripts/install_ros_source.sh

## Install ROS2 Humble
source ~/QUTMS/QUTMS_Driverless/installation/install_scripts/install_ros_humble.sh

## Package requirements
echo ""
echo "---Installing driverless package requirements---"
echo ""
sleep 3
pip install -r ~/QUTMS/QUTMS_Driverless/installation/requirements.txt

## Create an alias for ease
echo "alias a='source install/setup.bash'" >> ~/.bashrc

ln -s ~/QUTMS/QUTMS_Driverless/tools/qutms_cli_tools/qutms_cli_tools/build.sh ~/QUTMS/build.sh
ln -s ~/QUTMS/QUTMS_Driverless/tools/qutms_cli_tools/qutms_cli_tools/format.sh ~/QUTMS/format.sh
ln -s ~/QUTMS/QUTMS_Driverless/tools/qutms_cli_tools/qutms_cli_tools/launch.sh ~/QUTMS/launch.sh
ln -s ~/QUTMS/QUTMS_Driverless/tools/qutms_cli_tools/qutms_cli_tools/pull.sh ~/QUTMS/pull.sh
ln -s ~/QUTMS/QUTMS_Driverless/tools/qutms_cli_tools/qutms_cli_tools/record.sh ~/QUTMS/record.sh

## Check if user is going to work with vision
source ~/QUTMS/QUTMS_Driverless/installation/install_scripts/install_torch_pip.sh

## Source ROS
source /opt/ros/humble/setup.bash

## Install EUFS
source ~/QUTMS/QUTMS_Driverless/installation/install_scripts/install_eufs_sim.sh

## Install dependencies from src/
rosdep install --from-paths ~/QUTMS --ignore-src -r -y

## Pre commit for git
source ~/QUTMS/QUTMS_Driverless/installation/install_scripts/install_pre-commit.sh

## Check if user wants to install WSL version of GitKraken
source ~/QUTMS/QUTMS_Driverless/installation/install_scripts/install_gitkraken.sh

## Clone SBG driver
echo ""
echo "---Cloning SBG Driver---"
echo ""
cd ~/QUTMS
git clone https://github.com/SBG-Systems/sbg_ros2_driver.git

echo ""
echo "---Building packages---"
echo ""
sleep 3
cd ~/QUTMS

# build the rest of the workspace
source ~/QUTMS/build.sh

## Wrap up
echo "Thank you for installing, explore the rest of the wiki"
