#!/bin/bash

echo ""
echo "---Installing mambaforge---"
echo ""
sleep 3
cd ~
wget https://github.com/conda-forge/miniforge/releases/latest/download/Mambaforge-$(uname)-$(uname -m).sh
bash Mambaforge-$(uname)-$(uname -m).sh
source ~/mambaforge/bin/activate
conda config --set auto_activate_base false
conda deactivate
rm -rf Mambaforge-$(uname)-$(uname -m).sh
