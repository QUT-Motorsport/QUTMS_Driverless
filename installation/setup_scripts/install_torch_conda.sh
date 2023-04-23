#!/bin/bash

echo ""
echo "---Machine Learning Vision---"
echo ""
echo "Do you have an Nvidia GPU and are developing ML vision systems? ('yes' or 'no')"
read torch
echo ""
if [ $torch == "yes" ]; then
    cd ~
    mamba install -y pytorch torchvision pytorch-cuda=11.6 -c pytorch -c nvidia
    echo "export LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
    echo "Download any ML models from the Google Drive into the models folder (see wiki)"
    echo ""
    sleep 7
fi
