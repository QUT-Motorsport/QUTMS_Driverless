#!/bin/bash

cd ~

## Pre-reqs
sudo apt update
sudo apt upgrade
sudo apt install git
sudo apt install python3-pip
pip install pre-commit

## Clone Driverless repo
git clone --recurse-submodules -b not-quite-refactor https://github.com/QUT-Motorsport/QUTMS_Driverless.git

## Download and install mambaforge
wget https://github.com/conda-forge/miniforge/releases/latest/download/Mambaforge-$(uname)-$(uname -m).sh
bash Mambaforge-$(uname)-$(uname -m).sh
conda config --set auto_activate_base false

cd ~/QUTMS_Driverless/installation
pre-commit install

## Make directory for people to drop their rosbags in
mkdir bags/

## Create driverless development environment
mamba env create --name driverless_env --file humble_py39_dev_env.yml
conda config --env --add channels conda-forge
conda config --env --add channels robostack-humble

source ~/mambaforge/bin/activate
## Create an alias for ease
echo "alias a='conda activate driverless_env && . install/setup.bash'" >> ~/.bashrc

## Install package requirements
conda activate driverless_env

## Check if user is going to work with vision
echo "Do you have an Nvidia GPU and are developing ML vision systems? ('yes' or 'no')"
read torch
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

## Wrap up
echo "Thank you for installing, explore the rest of the wiki
