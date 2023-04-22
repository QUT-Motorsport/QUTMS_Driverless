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

## Download and install mambaforge
echo ""
echo "---Installing mambaforge---"
echo ""
sleep 3
wget https://github.com/conda-forge/miniforge/releases/latest/download/Mambaforge-$(uname)-$(uname -m).sh
bash Mambaforge-$(uname)-$(uname -m).sh
source ~/mambaforge/bin/activate
conda config --set auto_activate_base false
conda deactivate
rm -rf Mambaforge-$(uname)-$(uname -m).sh

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

## Create driverless development environment
echo ""
echo "---Creating driverless env---"
echo ""
sleep 3
mamba env create --name driverless_env --file installation/humble_py310_dev_env.yml
conda activate driverless_env
conda config --env --add channels conda-forge
conda config --env --add channels robostack-staging
mamba install -y --file installation/conda_requirements.txt

## Create an alias for ease
echo "alias a='conda activate driverless_env && source install/setup.bash'" >> ~/.bashrc

## Check if user is going to work with vision
echo ""
echo "---Machine Learning Vision---"
echo ""
echo "Do you have an Nvidia GPU and are developing ML vision systems? ('yes' or 'no')"
read torch
echo ""
if [ $torch == "yes" ]; then
    mamba install -y pytorch torchvision pytorch-cuda=11.6 -c pytorch -c nvidia
    echo "export LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
    echo "Download any ML models from the Google Drive into the models folder (see wiki)"
    echo ""
    sleep 7
fi

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

## Build initial driverless packages
echo ""
echo "---Building Driverless packages---"
echo ""
sleep 3
# colcon build --symlink-install --packages-up-to sim_translators mission_controller remote_control keyboard_control

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
