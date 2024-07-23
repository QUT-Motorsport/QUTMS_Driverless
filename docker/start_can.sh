#!/bin/bash

sudo modprobe can
sudo modprobe can_raw

sudo ip link set dev can0 down

sudo ip link set dev can0 type can bitrate 1000000
sudo ip link set dev can0 up

### THIS SCRIPT IS CALLED BY A STARTUP SERVICE ###
### FOUND IN /etc/systemd/system/can_startup.service ###
# [Unit]
# Description=Run CAN Startup Service
# DefaultDependencies=no
# After=network.target

# [Service]
# Type=simple
# ExecStart=/home/qutms/dev/repos/QUTMS_Driverless/docker.sh
# TimeoutStartSec=0
# RemainAfterExit=yes

# [Install]
# WantedBy=multi-user.target
