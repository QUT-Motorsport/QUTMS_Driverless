#!/bin/bash

export USERNAME=developer
export HOST_UID=$(shell id -u)

cd /home/qutms/dev/repos/QUTMS_Driverless/docker
/home/qutms/.local/bin/docker-compose -f ./docker-compose.yml -p QUTMS_Driverless run --rm jetson

### THIS SCRIPT IS CALLED BY A JETSON STARTUP SERVICE ###
### FOUND IN /etc/systemd/system/jetson_startup.service ###
# [Unit]
# Description=Jetson Startup Service
# DefaultDependencies=no
# After=network.target

# [Service]
# Type=simple
# User=qutms
# Group=qutms
# ExecStart=/home/qutms/dev/repos/QUTMS_Driverless/docker/jetson_startup.sh
# TimeoutStartSec=0
# RemainAfterExit=yes

# [Install]
# WantedBy=default.target
