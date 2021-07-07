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

# run airsim setup
WORKDIR /home/${USERNAME}/Formula-Student-Driverless-Simulator
RUN ./AirSim/setup.sh

# change to ros
WORKDIR /home/${USERNAME}/Formula-Student-Driverless-Simulator/ros

# install ros deps
RUN /bin/bash -c \
    "source /opt/ros/noetic/setup.bash; \
    rosdep update && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y &&\
    apt-get install -y --no-install-recommends \
        python3-catkin-tools \
        python3-osrf-pycommon \
        ros-noetic-cv-bridge \
        ros-noetic-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/"

# run a build
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; \
                 catkin init; \
                #  catkin config --extend /opt/ros/noetic; \
                 catkin build"


# ==============================================================================
#                    Build fs_msgs - ROS 2 Messages
# ==============================================================================
WORKDIR /home/${USERNAME}/ros_2_messages_ws
COPY --chown=${HOST_UID}:${HOST_GROUP} src/fs_msgs /home/${USERNAME}/ros_2_messages_ws/src/fs_msgs
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash; colcon build --packages-select fs_msgs"


# ==============================================================================
#                     Build ROS1 -> ROS2 bridge
# ==============================================================================
WORKDIR /home/${USERNAME}/bridge_ws/src
RUN git clone https://github.com/ros2/ros1_bridge.git && \
    cd .. && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash; \
    source /opt/ros/foxy/setup.bash; \
    source /home/${USERNAME}/Formula-Student-Driverless-Simulator/ros/devel/setup.bash; \
    source /home/${USERNAME}/ros_2_messages_ws/install/local_setup.bash; \
    colcon build --packages-select ros1_bridge --cmake-force-configure; \
    source /home/${USERNAME}/bridge_ws/install/local_setup.bash; \
    ros2 run ros1_bridge dynamic_bridge --print-pairs"

# set python3 as python command
RUN ln -s /usr/bin/python3 /usr/bin/python

# ==============================================================================
#                      Install src Dependencies
# ==============================================================================

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    # add more stuff here if needed
    && rm -rf /var/lib/apt/lists/

RUN pip3 install numpy
RUN pip3 install matplotlib


# ==============================================================================
#                      Install src Dependencies
# ==============================================================================

# make driverless workspace
RUN mkdir -p /home/${USERNAME}/driverless_ws/src && \
    chown -R ${USERNAME}:${HOST_GROUP} /home/${USERNAME}
WORKDIR /home/$USERNAME/driverless_ws

# copy in files and install dependencies
COPY --chown=${HOST_UID}:${HOST_GROUP} src/ /home/${USERNAME}/driverless_ws/src
RUN rosdep update && \
    apt-get update && \
    rosdep install -i --from-path src -y --rosdistro foxy -y \
    && rm -rf /var/lib/apt/lists/

# remove files
RUN rm -r /home/${USERNAME}/driverless_ws/src/*


# ==============================================================================
#                            Helpful Links
# ==============================================================================
# https://roscon.ros.org/2019/talks/roscon2019_bridging_ros1_to_ros2.pdf
# https://github.com/mabelzhang/ros1_bridge_sandbox/blob/master/docker/ros_melodic_dashing_deb_vrx/Dockerfile
# https://hub.docker.com/r/osrf/ros/tags?page=1&ordering=last_updated
# https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator
