#!/bin/bash

cd ~

## Pre-reqs
echo ""
echo "---Installing pre-reqs---"
echo ""
sleep 3
sudo apt update
sudo apt upgrade -y
sudo apt install -y git python3-pip cmake

# for installing pip rosdeps
export PIP_BREAK_SYSTEM_PACKAGES=1

## Make workspace
mkdir ~/QUTMS
cd ~/QUTMS
echo "export QUTMS_WS=~/QUTMS" >> ~/.bashrc
export QUTMS_WS=~/QUTMS

# directory for people to drop their rosbags in
mkdir bags/

## Clone Driverless repo
echo ""
echo "---Cloning Driverless repo---"
echo ""
sleep 3
git clone -b feature/jazzy_migration --recurse-submodules https://github.com/QUT-Motorsport/QUTMS_Driverless.git
cd QUTMS_Driverless

## Set ROS2 repositories
source ~/QUTMS/QUTMS_Driverless/installation/install_scripts/install_ros_source.sh

## Install ROS2
source ~/QUTMS/QUTMS_Driverless/installation/install_scripts/install_ros_jazzy.sh

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
ln -s ~/QUTMS/QUTMS_Driverless/tools/play_bag.sh ~/QUTMS/play_bag.sh

## Source ROS
source /opt/ros/jazzy/setup.bash

## Install dependencies from src/
rosdep install --from-paths ~/QUTMS --ignore-src -r -y

## Pre commit for git
source ~/QUTMS/QUTMS_Driverless/installation/install_scripts/install_pre-commit.sh

## Check if user wants to install WSL version of GitKraken
source ~/QUTMS/QUTMS_Driverless/installation/install_scripts/install_gitkraken.sh

echo ""
echo "---Building packages---"
echo ""
sleep 3
cd ~/QUTMS

# build the rest of the workspace
source ~/QUTMS/build.sh --all

## Wrap up
echo "Thank you for installing, explore the rest of the wiki"
