FROM stereolabs/zed:3.6-tools-devel-jetson-jp4.6


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
