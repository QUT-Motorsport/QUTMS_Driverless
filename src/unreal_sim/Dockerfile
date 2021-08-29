FROM osrf/ros:foxy-desktop

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

WORKDIR /root
RUN git clone https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator.git --recurse-submodules

# run airsim setup
WORKDIR /root/Formula-Student-Driverless-Simulator
RUN AirSim/setup.sh

# change to ros
WORKDIR /root/Formula-Student-Driverless-Simulator/ros

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
RUN /bin/bash -c \
    "source /opt/ros/noetic/setup.bash; \
    catkin init; \
    catkin build"


# ==============================================================================
#                    Build fs_msgs - ROS 2 Messages
# ==============================================================================

WORKDIR /root/ros_2_messages_ws
COPY src/unreal_sim/fs_msgs /ros_2_messages_ws/src/fs_msgs
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash; colcon build --packages-select fs_msgs"


# ==============================================================================
#                        Build ROS1 -> ROS2 bridge
# ==============================================================================

WORKDIR /root/bridge_ws/src
RUN git clone https://github.com/ros2/ros1_bridge.git && \
    cd .. && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash; \
    source /opt/ros/foxy/setup.bash; \
    source /root/Formula-Student-Driverless-Simulator/ros/devel/setup.bash; \
    source /root/ros_2_messages_ws/install/local_setup.bash; \
    colcon build --packages-select ros1_bridge --cmake-force-configure; \
    source /root/bridge_ws/install/local_setup.bash; \
    ros2 run ros1_bridge dynamic_bridge --print-pairs"


# ==============================================================================
#                              Dev Tools
# ==============================================================================

# set python3 as python command
RUN ln -s /usr/bin/python3 /usr/bin/python


# ==============================================================================
#                             Copy run file
# ==============================================================================

COPY src/unreal_sim/start.sh /start.sh
RUN chmod +x /start.sh
