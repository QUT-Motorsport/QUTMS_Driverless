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
echo ""
echo "---Setting up ROS2 repositories---"
echo ""
sleep 3
sudo apt install -y software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

## Install ROS2 Humble
echo ""
echo "---Installing ROS2 Humble---"
echo ""
sleep 3
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools
sudo rosdep init
rosdep update
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

mkdir ~/QUTMS
cd ~/QUTMS

## Make directory for people to drop their rosbags in
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
echo ""
echo "---Machine Learning Vision---"
echo ""
echo "Do you have an Nvidia GPU and are developing ML vision systems? ('yes' or 'no')"
read torch
echo ""
if [ $torch == "yes" ]; then
    pip install torch torchvision --extra-index-url https://download.pytorch.org/whl/cu117
    echo "export LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
    echo ""
    echo "Download any ML models from the Google Drive into the models folder (see wiki)"
    echo ""
    sleep 7
fi

## Source ROS
source /opt/ros/humble/setup.bash

## Install EUFS
echo ""
echo "---Installing EUFS---"
echo ""
sleep 3
cd ~/QUTMS
git clone https://github.com/QUT-Motorsport/eufs_sim.git
git clone https://gitlab.com/eufs/eufs_msgs.git
git clone https://github.com/QUT-Motorsport/eufs_rviz_plugins.git
git clone https://github.com/QUT-Motorsport/qutms_msgs.git
echo "export EUFS_MASTER=~/QUTMS" >> ~/.bashrc
export EUFS_MASTER=~/QUTMS
export QUTMS_WS=~/QUTMS

## Install dependencies from src/
rosdep install --from-paths ~/QUTMS --ignore-src -r -y

echo ""
echo "---Building packages---"
echo ""
sleep 3
colcon build --symlink-install --packages-ignore 

## Pre commit for git
echo ""
echo "---Installing pre-commit---"
echo ""
sleep 3
pre-commit install

## Check if user wants to install WSL version of GitKraken
echo ""
echo "---GitKraken---"
echo ""
echo "Do you wish to use Gitkraken to manage your repo? If so it is recommended to install the WSL version now ('yes' or 'no')"
read gitkraken
echo ""
if [ $gitkraken == "yes" ]; then
    wget https://release.gitkraken.com/linux/gitkraken-amd64.deb
    sudo dpkg -i ./gitkraken-amd64.deb
    sudo apt-get install -f -y
    rm gitkraken-amd64.deb
    echo ""
fi

## Wrap up
echo "Thank you for installing, explore the rest of the wiki"
