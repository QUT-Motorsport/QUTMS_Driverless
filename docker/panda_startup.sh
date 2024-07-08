#!/bin/bash

cd /home/qev3d/dev/repos/QUTMS_Driverless/docker
make run target=panda

### THIS SCRIPT IS CALLED BY A ROSCUBE STARTUP SERVICE ###
### FOUND IN /etc/systemd/system/panda_startup.service ###
# [Unit]
# Description=Run Panda Startup Service as user qev3d
# DefaultDependencies=no
# After=network.target

# [Service]
# Type=simple
# User=qev3d
# Group=qev3d
# ExecStart=/home/qev3d/dev/repos/QUTMS_Driverless/docker/panda_startup.sh
# TimeoutStartSec=0
# RemainAfterExit=yes

# [Install]
# WantedBy=default.target
