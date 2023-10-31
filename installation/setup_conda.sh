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
source ~/QUTMS/QUTMS_Driverless/installation/install_scripts/install_mambaforge.sh

## Create driverless development environment
source ~/QUTMS/QUTMS_Driverless/installation/install_scripts/install_conda_env.sh

## Create an alias for ease
echo "alias a='conda activate driverless_env && source install/setup.bash'" >> ~/.bashrc

## Check if user is going to work with vision
source ~/QUTMS/QUTMS_Driverless/installation/install_scripts/install_torch_conda.sh

## Install EUFS
source ~/QUTMS/QUTMS_Driverless/installation/install_scripts/install_eufs_sim.sh

## Pre commit for git
source ~/QUTMS/QUTMS_Driverless/installation/install_scripts/install_pre-commit.sh

## Check if user wants to install WSL version of GitKraken
source ~/QUTMS/QUTMS_Driverless/installation/install_scripts/install_gitkraken.sh

## Clone SBG driver
echo ""
echo "---Cloning SBG Driver---"
echo ""
sleep 3
cd ~/QUTMS
git clone https://github.com/SBG-Systems/sbg_ros2_driver.git

echo ""
echo "---Building packages---"
echo ""
sleep 3
cd ~/QUTMS
colcon build --symlink-install --packages-up-to qutms_cli_tools
# once the cli tools are built, we can use them to build the rest of the workspace
ws_build --all

## Wrap up
echo "Thank you for installing, explore the rest of the wiki"
