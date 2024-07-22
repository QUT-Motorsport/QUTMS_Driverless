#!/bin/bash

LOG_FILE="/home/qev3d/dev/repos/QUTMS_Driverless/tools/startup.log"
IMAGE_NAME="qutms_driverless_panda"  # Define monitored image

# Record the time the system booted up
BOOT_TIME=$(date +%s)

# Function to get the container ID by image name
get_container_id_by_image() {
    docker ps -q --filter "ancestor=$IMAGE_NAME"
}

# Wait until the Docker container is running
CONTAINER_ID=$(get_container_id_by_image)
while [ -z "$CONTAINER_ID" ] || [ "$(docker inspect -f '{{.State.Running}}' $CONTAINER_ID 2>/dev/null)" != "true" ]; do
    if [ -z "$CONTAINER_ID" ]; then
        echo "No container found for image $IMAGE_NAME. Retrying..." >> $LOG_FILE
    fi
    sleep 1
    CONTAINER_ID=$(get_container_id_by_image)
done

# Record the time the container started
CONTAINER_START_TIME=$(date +%s)

# Calculate the time difference
TIME_DIFF=$((CONTAINER_START_TIME - BOOT_TIME))

# Log the results
echo "Container with image $IMAGE_NAME started in $TIME_DIFF seconds after system boot." >> $LOG_FILE
echo "Service Successfully Logging..." >> $LOG_FILE
