#!/bin/bash

echo ""
echo "---Machine Learning Vision---"
echo ""
echo "Do you have an Nvidia GPU and are developing ML vision systems? ('yes' or 'no')"
read torch
echo ""
if [ $torch == "yes" ]; then
    pip install torch torchvision --extra-index-url https://download.pytorch.org/whl/cu117
    echo "export LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
    echo ""
    echo "Download any ML models from the Google Drive into the models folder (see wiki)"
    echo ""
    sleep 7
fi
