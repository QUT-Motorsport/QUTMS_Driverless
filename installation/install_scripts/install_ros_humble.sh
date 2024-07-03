#!/bin/bash

echo ""
echo "---Installing ROS2 Humble---"
echo ""
sleep 3
cd ~
sudo apt update
sudo apt install -y ros-humble-desktop-full ros-dev-tools
sudo rosdep init
rosdep update
rosdep install -y \
    --rosdistro=${ROS_DISTRO} \
    --ignore-src \
    --from-paths QUTMS_Driverless/src \
    --skip-keys "sbg_driver zed_ros2"
    # skip the 3rd party device drivers that get installed on-car (we dont have the devices)

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
