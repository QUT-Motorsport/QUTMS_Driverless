source "/home/developer/mambaforge/bin/activate"
conda activate driverless_env

# REMEMBER to update deps in conda_requirements as well
# (these deps are the same as conda_requirements with the exception of pytorch and torchvision)
mamba install -y \
    opencv==4.5.2 \
    requests==2.26.0
