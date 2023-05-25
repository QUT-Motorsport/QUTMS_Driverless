#!/bin/bash

echo ""
echo "---Installing ROS2 Humble---"
echo ""
sleep 3
cd ~
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools
sudo rosdep init
rosdep update
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
