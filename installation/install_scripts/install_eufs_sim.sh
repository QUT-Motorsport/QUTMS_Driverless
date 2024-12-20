#!/bin/bash

echo ""
echo "---Installing EUFS---"
echo ""
sleep 3
cd ~/QUTMS
git clone https://github.com/QUT-Motorsport/eufs_sim.git
echo "export EUFS_MASTER=~/QUTMS" >> ~/.bashrc
echo "export QUTMS_WS=~/QUTMS" >> ~/.bashrc
export EUFS_MASTER=~/QUTMS
export QUTMS_WS=~/QUTMS

## Clone SBG driver
## DEPRECATED: SBG driver can now be installed via rosdep
# echo ""
# echo "---Cloning SBG Driver---"
# echo ""
# git clone https://github.com/SBG-Systems/sbg_ros2_driver.git

sudo rosdep init
rosdep update
rosdep install -y \
    --rosdistro=${ROS_DISTRO} \
    --ignore-src \
    --from-paths ~/QUTMS \
    --skip-keys "zed_ros2"
