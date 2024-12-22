#!/bin/bash

echo ""
echo "---Installing EUFS---"
echo ""
sleep 3
cd ~/QUTMS
git clone https://github.com/QUT-Motorsport/eufs_sim.git
echo "export EUFS_MASTER=~/QUTMS" >> ~/.bashrc
export EUFS_MASTER=~/QUTMS

sudo rosdep init
rosdep update
rosdep install -y \
    --rosdistro=${ROS_DISTRO} \
    --ignore-src \
    --from-paths ~/QUTMS \
    --skip-keys "zenoh_bridge_ros2dds"
