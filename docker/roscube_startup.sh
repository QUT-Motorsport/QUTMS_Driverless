#!/bin/bash

cd /home/qutms/dev/repos/QUTMS_Driverless/docker
make run target=roscube

### THIS SCRIPT IS CALLED BY A ROSCUBE STARTUP SERVICE ###
### FOUND IN /etc/systemd/system/roscube_startup.service ###
# [Unit]
# Description=Run Roscube Startup Service as user qutms
# DefaultDependencies=no
# After=network.target

# [Service]
# Type=simple
# User=qutms
# Group=qutms
# ExecStart=/home/qutms/dev/repos/QUTMS_Driverless/docker/roscube_startup.sh
# TimeoutStartSec=0
# RemainAfterExit=yes

# [Install]
# WantedBy=default.target
