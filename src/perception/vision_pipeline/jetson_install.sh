source "/home/developer/mambaforge/bin/activate"
conda activate driverless_env

apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-numpy \
    python3-setuptools \
    libpython3-dev \
    libopenblas-base \
    libopenmpi-dev \
    libjpeg-dev zlib1g-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev

cd /home/developer
wget https://nvidia.box.com/shared/static/fjtbno0vpo676a25cgvuqc1wty0fkkg6.whl \
    -O torch-1.10.0-cp36-cp36m-linux_aarch64.whl
pip3 install torch-1.10.0-cp36-cp36m-linux_aarch64.whl

git clone --branch v0.11.1 https://github.com/pytorch/vision torchvision

cd torchvision
export BUILD_VERSION=0.11.1;
export LD_LIBRARY_PATH=/usr/local/cuda/lib64 ${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}};
python3 setup.py install --user

# REMEMBER to update deps in conda_requirements as well
# (these deps are the same as conda_requirements with the exception of pytorch and torchvision)
mamba install -y \
    opencv==4.5.2 \
    numpy==1.21.4 \
    matplotlib==3.5.1 \
    requests==2.26.0 \
    tqdm>=4.41.0 \
    pandas>=1.1.4 \
    seaborn>=0.11.0 \