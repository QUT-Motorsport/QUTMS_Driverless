#!/bin/bash

LOG_FILE="home/qev3d/dev/repos/QUTMS_Driverless/tools/startup.log"
CONTAINER_NAME="qutms_driverless_panda"

# Record the time the system booted up
BOOT_TIME=$(date +%s)

# Wait until the Docker container is running
until [ "$(docker inspect -f '{{.State.Running}}' $CONTAINER_NAME)" == "true" ]; do
    sleep 1
done

# Record the time the container started
CONTAINER_START_TIME=$(date +%s)

# Calculate the time difference
TIME_DIFF=$((CONTAINER_START_TIME - BOOT_TIME))

# Log the resultx
echo "Container $CONTAINER_NAME started in $TIME_DIFF seconds after system boot." >> $LOG_FILE
echo "Service Succesfully Logging..." >> $LOG_FILE