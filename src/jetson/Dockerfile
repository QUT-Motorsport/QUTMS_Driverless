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


# ==============================================================================
#                           Override udev rules
#             (allows ${USERNAME} user to open the ZED Camera)
# ==============================================================================
RUN cat > /etc/udev/rules.d/99-slabs.rules << EOF \
# --- begin 99-slabs.rules ---
\# HIDAPI/libusb
SUBSYSTEM=="usb", ATTRS{idVendor}=="2b03", ATTRS{idProduct}=="f681", MODE="0666", OWNER=${USERNAME} \
SUBSYSTEM=="usb", ATTRS{idVendor}=="2b03", ATTRS{idProduct}=="f781", MODE="0666", OWNER=${USERNAME} \
SUBSYSTEM=="usb", ATTRS{idVendor}=="2b03", ATTRS{idProduct}=="f881", MODE="0666", OWNER=${USERNAME} \
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE="0666", OWNER=${USERNAME} \
\
\# HIDAPI/hidraw \
KERNEL=="hidraw*", ATTRS{busnum}=="1", ATTRS{idVendor}=="2b03", ATTRS{idProduct}=="f681", MODE="0666", OWNER=${USERNAME} \
KERNEL=="hidraw*", ATTRS{busnum}=="1", ATTRS{idVendor}=="2b03", ATTRS{idProduct}=="f781", MODE="0666", OWNER=${USERNAME} \
KERNEL=="hidraw*", ATTRS{busnum}=="1", ATTRS{idVendor}=="2b03", ATTRS{idProduct}=="f881", MODE="0666", OWNER=${USERNAME} \
KERNEL=="hidraw*", ATTRS{busnum}=="1", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE="0666", OWNER=${USERNAME} \
\
\# blacklist for usb autosuspend \
\# http://kernel.org/doc/Documentation/usb/power-management.txt \
ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="2b03", ATTRS{idProduct}=="f780", TEST=="power/control", ATTR{power/control}="on" \
ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="2b03", ATTRS{idProduct}=="f781", TEST=="power/control", ATTR{power/control}="on" \
ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="2b03", ATTRS{idProduct}=="f881", TEST=="power/control", ATTR{power/control}="on" \
ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="0424", ATTRS{idProduct}=="2512", TEST=="power/control", ATTR{power/control}="on" \
\
ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="2b03", ATTRS{idProduct}=="f780", TEST=="power/autosuspend", ATTR{power/autosuspend}="-1" \
ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="2b03", ATTRS{idProduct}=="f781", TEST=="power/autosuspend", ATTR{power/autosuspend}="-1" \
ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="2b03", ATTRS{idProduct}=="f881", TEST=="power/autosuspend", ATTR{power/autosuspend}="-1" \
ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="0424", ATTRS{idProduct}=="2512", TEST=="power/autosuspend", ATTR{power/autosuspend}="-1" \
\
ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="2b03", ATTRS{idProduct}=="f780", TEST=="power/autosuspend_delay_ms", ATTR{power/autosuspend_delay_ms}="-1" \
ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="2b03", ATTRS{idProduct}=="f781", TEST=="power/autosuspend_delay_ms", ATTR{power/autosuspend_delay_ms}="-1" \
ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="2b03", ATTRS{idProduct}=="f881", TEST=="power/autosuspend_delay_ms", ATTR{power/autosuspend_delay_ms}="-1" \
ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="0424", ATTRS{idProduct}=="2512", TEST=="power/autosuspend_delay_ms", ATTR{power/autosuspend_delay_ms}="-1" \
# --- end 99-slabs.rules ---
EOF
