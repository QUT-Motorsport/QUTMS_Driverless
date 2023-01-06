#!/bin/bash

cd ~

## Pre-reqs
sudo apt update
sudo apt upgrade -y
sudo apt install -y git python3-pip pre-commit
pip install pre-commit

## Download and install mambaforge
wget https://github.com/conda-forge/miniforge/releases/latest/download/Mambaforge-$(uname)-$(uname -m).sh
bash Mambaforge-$(uname)-$(uname -m).sh
conda config --set auto_activate_base false
rm -rf Mambaforge-$(uname)-$(uname -m).sh

## Clone Driverless repo
git clone --recurse-submodules -b not-quite-refactor https://github.com/QUT-Motorsport/QUTMS_Driverless.git
cd ~/QUTMS_Driverless

## Make directory for people to drop their rosbags in
mkdir bags/

## Create driverless development environment
cd ~/QUTMS_Driverless/installation
mamba env create --name driverless_env --file humble_py39_dev_env.yml
conda config --env --add channels conda-forge
conda config --env --add channels robostack-humble

source ~/mambaforge/bin/activate
## Create an alias for ease
echo "alias a='conda activate driverless_env && source install/setup.bash'" >> ~/.bashrc

## Install package requirements
conda activate driverless_env

## Check if user is going to work with vision
echo ""
echo "Do you have an Nvidia GPU and are developing ML vision systems? ('yes' or 'no')"
echo ""
read torch
echo ""
if [ $torch == "yes" ]; then
    mamba install -y pytorch torchvision pytorch-cuda=11.6 -c pytorch -c nvidia
    echo "export LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
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
colcon build

## Build initial driverless packages
cd ~/QUTMS_Driverless
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
