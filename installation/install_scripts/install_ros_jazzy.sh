#!/bin/bash

echo ""
echo "---Installing ROS2 Jazzy---"
echo ""
sleep 3
cd ~/QUTMS
sudo apt update
sudo apt install -y ros-jazzy-desktop-full ros-dev-tools
sudo rosdep init
rosdep update
rosdep install -y \
    --rosdistro=${ROS_DISTRO} \
    --ignore-src \
    --from-paths QUTMS_Driverless/src \
    --skip-keys "zenoh_bridge_ros2dds"

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
