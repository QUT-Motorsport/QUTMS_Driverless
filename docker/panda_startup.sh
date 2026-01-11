#!/bin/bash

cd /home/qutms/repos/QUTMS_Driverless/docker
make run target=panda

### THIS SCRIPT IS CALLED BY A ROSCUBE STARTUP SERVICE ###
### FOUND IN /etc/systemd/system/panda_startup.service ###
# [Unit]
# Description=Run Panda Startup Service as user qev3d
# DefaultDependencies=no
# After=docker.service

# [Service]
# Type=simple
# User=qutms
# Group=qutms
# ExecStart=/home/qutms/repos/QUTMS_Driverless/docker/panda_startup.sh
# TimeoutStartSec=0
# RemainAfterExit=yes

# [Install]
# WantedBy=default.target
