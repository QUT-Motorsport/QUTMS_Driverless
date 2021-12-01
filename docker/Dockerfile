FROM osrf/ros:foxy-desktop

# Docker config setup
ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"]


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
#                          Install Dev Tools
# ==============================================================================

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    nano \
    # add more stuff here if needed
    && rm -rf /var/lib/apt/lists/

# set python3 as python command
RUN ln -s /usr/bin/python3 /usr/bin/python

# Install rosboard https://github.com/dheera/rosboard
WORKDIR /home/$USERNAME
RUN pip3 install tornado simplejpeg
RUN git clone https://github.com/dheera/rosboard.git


# ==============================================================================
#                           Add ROS entrypoint
# ==============================================================================
# setup entrypoint
COPY ./docker/ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
RUN echo "source /opt/ros/foxy/setup.bash" >> /home/$USERNAME/.bashrc


# ==============================================================================
#                         Install src Dependencies
# ==============================================================================

# make driverless workspace
RUN mkdir -p /home/${USERNAME}/driverless_ws/src && \
    chown -R ${USERNAME}:${HOST_GROUP} /home/${USERNAME}
WORKDIR /home/$USERNAME/driverless_ws

# copy in files and install dependencies
COPY --chown=${HOST_UID}:${HOST_GROUP} ./src /home/${USERNAME}/driverless_ws/src
RUN rosdep update && \
    apt-get update && \
    rosdep install -i --from-path src -y --rosdistro foxy -y \
    && rm -rf /var/lib/apt/lists/

# run a build
RUN source /opt/ros/foxy/setup.bash; \
    colcon build --symlink-install --packages-up-to driverless_msgs

# remove files
RUN rm -r /home/${USERNAME}/driverless_ws/src/*
