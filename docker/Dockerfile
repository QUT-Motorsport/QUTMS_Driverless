FROM osrf/ros:foxy-desktop


# ==============================================================================
#                           Set up a user using host UID
#    (this helps with file permissions when mounting folders into a container)
# ==============================================================================
ARG USERNAME
ARG HOST_UID
# dialout group is used for compatability with mac
ARG HOST_GROUP=dialout

# create user and set no password required
RUN useradd \
        -r \
        -m \
        -d /home/${USERNAME}/ \
        -u ${HOST_UID} \
        -g ${HOST_GROUP} \
        -G sudo,video,audio \
        ${USERNAME} && \
    echo ${USERNAME}:${USERNAME} | chpasswd && \
    echo "${USERNAME} ALL=NOPASSWD: ALL" >> /etc/sudoers.d/${USERNAME}


# ==============================================================================
#                      Install ROS 1 Noetic
# ==============================================================================
# Set up sources.list
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list'

# Set up keys
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install Melodic
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base \
    && rm -rf /var/lib/apt/lists/*

ENV ROS1_DISTRO noetic
ENV ROS2_DISTRO foxy


# ==============================================================================
#                    Install FS Sim - ROS 1 Messages
# ==============================================================================
# install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    wget \
    libyaml-cpp-dev \
    libcurl4-openssl-dev \
    && rm -rf /var/lib/apt/lists/*

# install git lfs
RUN curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash

# FS Sim must be installed under $HOME
WORKDIR /home/${USERNAME}
RUN git clone https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator.git --recurse-submodules

# run setup
WORKDIR /home/${USERNAME}/Formula-Student-Driverless-Simulator
RUN ./AirSim/setup.sh

# install ros deps
RUN /bin/bash -c \
    "source /opt/ros/noetic/setup.bash; \
    cd /home/${USERNAME}/Formula-Student-Driverless-Simulator/ros && \
    rosdep update && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y &&\
    apt-get install -y --no-install-recommends \
        ros-noetic-cv-bridge \
        ros-noetic-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/"

WORKDIR /home/${USERNAME}/Formula-Student-Driverless-Simulator/ros
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make_isolated --install"


# ==============================================================================
#                    Build fs_msgs - ROS 2 Messages
# ==============================================================================

# WORKDIR /home/${USERNAME}/ros_2_messages_ws
# COPY --chown=${HOST_UID}:${HOST_GROUP} src/fs_msgs /home/${USERNAME}/ros_2_messages_ws/src
# RUN /bin/bash -c "source /opt/ros/foxy/setup.bash; colcon build --packages-select fs_msgs"


# https://github.com/osrf/docker_images/blob/e95025e4b17349438a4b68951bd6d8702b0020be/ros/foxy/ubuntu/focal/ros1-bridge/Dockerfile

# NEXT STEP:
# Clone this into the correct place
# https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator

# Create a package in src that mirrors the message (or get the package they use?)
# https://github.com/FS-Driverless/fs_msgs - see if this will compile on ROS 2

# Use the container to compile the mappings

# https://roscon.ros.org/2019/talks/roscon2019_bridging_ros1_to_ros2.pdf
# https://github.com/mabelzhang/ros1_bridge_sandbox/blob/master/docker/ros_melodic_dashing_deb_vrx/Dockerfile
# https://hub.docker.com/r/osrf/ros/tags?page=1&ordering=last_updated
# https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator

# Cant do this :(
# https://stackoverflow.com/questions/5303496/how-to-change-a-git-submodule-to-point-to-a-subfolder