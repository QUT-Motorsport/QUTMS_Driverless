FROM ros:humble

ENV ROS_DISTRO=humble

# Docker config setup
ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"]

WORKDIR /tmp

# ==============================================================================
#                           Set up a user using host UID
#    (this helps with file permissions when mounting folders into a container)
# ==============================================================================

ARG USERNAME
ARG UID
# dialout group is used for compatability with mac
ARG GID=dialout

# create user and set no password required
RUN groupadd --gid $GID $USERNAME
RUN useradd \
        -r \
        -m \
        -d /home/${USERNAME}/ \
        -u ${UID} \
        -g ${GID} \
        -G sudo,video,audio \
        ${USERNAME} && \
    echo ${USERNAME}:${USERNAME} | chpasswd && \
    echo "${USERNAME} ALL=NOPASSWD: ALL" >> /etc/sudoers


RUN mkdir -p /home/${USERNAME}/driverless_ws/src && \
    mkdir -p /home/${USERNAME}/driverless_ws/tools && \
    chown -R ${USERNAME}:${GID} /home/${USERNAME}/driverless_ws

# ==============================================================================
#                      Download repos to install rosdeps
# ==============================================================================

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        can-utils net-tools usbutils udev mesa-common-dev \
        python3-pip

COPY ./QUTMS_Driverless/src /tmp/src
COPY ./slam_toolbox /tmp/slam_toolbox
COPY ./sbg_ros2_driver /tmp/sbg_ros2_driver

# RUN git clone https://github.com/SBG-Systems/sbg_ros2_driver.git
# RUN git clone https://github.com/SteveMacenski/slam_toolbox.git
# # specific commit for slam_toolbox that builds with humble and lifecycle nodes
# RUN cd slam_toolbox && git checkout a3442d2f6824ff058fab0cb0a635e7a454294855 && cd ..

# # install ros requirements
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep update && rosdep install -y \
        --rosdistro ${ROS_DISTRO} \
        --from-paths src slam_toolbox sbg_ros2_driver \
        --ignore-src

# install pip requirements
COPY ./QUTMS_Driverless/docker/requirements.txt /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements.txt && \
    rm /tmp/requirements.txt

RUN rm -rf /var/lib/apt/lists/* && apt-get clean

RUN python3 src/navigation/planners/scripts/pre_calc.py

# remove repos as deps have now been installed
RUN rm -rf src sbg_ros2_driver slam_toolbox

# ==============================================================================
#                          Build and initialise ROS 2
# ==============================================================================

USER ${USERNAME}
WORKDIR /home/${USERNAME}/driverless_ws

COPY ./QUTMS_Driverless/docker/cyclonedds.xml ./cyclonedds.xml
ENV CYCLONEDDS_URI=file:///home/developer/driverless_ws/cyclonedds.xml 
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=69
ENV QUTMS_WS=~/driverless_ws

COPY ./QUTMS_Driverless/docker/ros_entrypoint.sh ./ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
