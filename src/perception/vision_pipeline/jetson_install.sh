source "/home/developer/mambaforge/bin/activate"
conda activate driverless_env

# install pytorch
# https://github.com/pytorch/pytorch#from-source
git clone --recursive https://github.com/pytorch/pytorch
cd pytorch
mamba install -y \
    astunparse \
    numpy \
    ninja \
    pyyaml \
    setuptools \
    cmake \
    cffi \
    typing_extensions \
    future \
    six \
    requests \
    dataclasses

# set up gcc and g++ for CUDA 10.2 (requires gcc 8)
# https://stackoverflow.com/a/46380601
MAX_GCC_VERSION=8
apt-get update && apt-get install -y gcc-$MAX_GCC_VERSION g++-$MAX_GCC_VERSION
export CUDAHOSTCXX=/usr/bin/gcc-$MAX_GCC_VERSION

# run install
export CMAKE_PREFIX_PATH=${CONDA_PREFIX:-"$(dirname $(which conda))/../"}
python3 setup.py install

# install torch vision
git clone --branch v0.11.2 https://github.com/pytorch/vision torchvision
cd torchvision
export BUILD_VERSION=0.11.2
export LD_LIBRARY_PATH=/usr/local/cuda/lib64 ${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
python3 setup.py install

# REMEMBER to update deps in conda_requirements as well
# (these deps are the same as conda_requirements with the exception of pytorch and torchvision)
mamba install -y \
    opencv==4.5.2 \
    numpy==1.21.4 \
    matplotlib==3.5.1 \
    requests==2.26.0 \
    tqdm>=4.41.0 \
    pandas>=1.1.4 \
    seaborn>=0.11.0
