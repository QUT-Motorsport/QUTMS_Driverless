#!/bin/bash

echo ""
echo "---Installing EUFS---"
echo ""
sleep 3
cd ~/QUTMS
git clone https://github.com/QUT-Motorsport/eufs_sim.git
git clone https://gitlab.com/eufs/eufs_msgs.git
git clone https://github.com/QUT-Motorsport/eufs_rviz_plugins.git
# git clone https://github.com/QUT-Motorsport/qutms_msgs.git
echo "export EUFS_MASTER=~/QUTMS" >> ~/.bashrc
echo "export QUTMS_WS=~/QUTMS" >> ~/.bashrc
export EUFS_MASTER=~/QUTMS
export QUTMS_WS=~/QUTMS
