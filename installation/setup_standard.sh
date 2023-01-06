#!/bin/bash

cd ~

## Pre-reqs
sudo apt update
sudo apt upgrade -y
sudo apt install -y git python3-pip pre-commit

## Set ROS2 repositories
sudo apt install -y software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

## Install ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools
sudo rosdep init
rosdep update
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

## Clone Driverless repo
git clone --recurse-submodules -b not-quite-refactor https://github.com/QUT-Motorsport/QUTMS_Driverless.git
cd ~/QUTMS_Driverless

## Make directory for people to drop their rosbags in
mkdir bags/

## Package requirements
pip install -r installation/requirements.txt

## Create an alias for ease
echo "alias a='source install/setup.bash'" >> ~/.bashrc

## Check if user is going to work with vision
echo "Do you have an Nvidia GPU and are developing ML vision systems? ('yes' or 'no')"
read torch
if [ $torch == "yes" ]; then
    pip install torch torchvision --extra-index-url https://download.pytorch.org/whl/cu117
    echo "export LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
    echo ""
    echo "Download any ML models from the Google Drive into the models folder (see wiki)"
    sleep 7
fi

## Install FSDS
cd ~
git clone --recurse-submodules https://github.com/QUT-Motorsport/Formula-Student-Driverless-Simulator.git
cd ~/Formula-Student-Driverless-Simulator
AirSim/setup.sh

## Build FSDS package
cd ~/Formula-Student-Driverless-Simulator/ros2
## Install dependencies from src/
rosdep install --from-paths src -y --ignore-src
colcon build

## Build initial driverless packages
cd ~/QUTMS_Driverless
## Install dependencies from src/
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install --packages-up-to sim_translators mission_controller remote_control keyboard_control

## Pre commit for git
pre-commit install

## Check if user wants to install WSL version of GitKraken
echo "Do you wish to use Gitkraken to manage your repo? If so it is recommended to install the WSL version now ('yes' or 'no')"
read gitkraken
if [ $gitkraken == "yes" ]; then
    wget https://release.gitkraken.com/linux/gitkraken-amd64.deb
    sudo dpkg -i ./gitkraken-amd64.deb
    sudo apt-get install -f -y
    rm gitkraken-amd64.deb
fi

## Wrap up
echo "Thank you for installing, explore the rest of the wiki"
