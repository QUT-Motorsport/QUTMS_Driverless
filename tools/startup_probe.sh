#!/bin/bash

LOG_FILE="/home/qev3d/dev/repos/QUTMS_Driverless/tools/startup.log"
IMAGE_NAME="qutms_driverless_panda"  # Define monitored image

# Record the time the system booted up
BOOT_TIME=$(date +%s)

# Function to log service start time with timestamp
log_service_start_time() {
    SERVICE_NAME=$1
    systemctl is-active --quiet $SERVICE_NAME
    while [ $? -ne 0 ]; do
        sleep 1
        systemctl is-active --quiet $SERVICE_NAME
    done
    SERVICE_START_TIME=$(date +%s)
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $SERVICE_NAME started in $((SERVICE_START_TIME - BOOT_TIME)) seconds after system boot." >> $LOG_FILE
}

# Log start times for the specified services
SERVICES=("NetworkManager.service" "systemd-networkd.service" "network-online.target" "docker.socket" "firewalld.service" "containerd.service" "time-set.target")
for SERVICE in "${SERVICES[@]}"; do
    log_service_start_time $SERVICE &
done

# Function to get the container ID by image name
get_container_id_by_image() {
    docker ps -q --filter "ancestor=$IMAGE_NAME"
}

# Wait until the Docker container is running
CONTAINER_ID=$(get_container_id_by_image)
while [ -z "$CONTAINER_ID" ] || [ "$(docker inspect -f '{{.State.Running}}' $CONTAINER_ID 2>/dev/null)" != "true" ]; do
    if [ -z "$CONTAINER_ID" ]; then
        echo "$(date '+%Y-%m-%d %H:%M:%S') - No container found for image $IMAGE_NAME. Retrying..." >> $LOG_FILE
    fi
    sleep 1
    CONTAINER_ID=$(get_container_id_by_image)
done

# Record the time the container started
CONTAINER_START_TIME=$(date +%s)

# Calculate the time difference
TIME_DIFF=$((CONTAINER_START_TIME - BOOT_TIME))

# Log the results
echo "$(date '+%Y-%m-%d %H:%M:%S') - Container with image $IMAGE_NAME started in $TIME_DIFF seconds after system boot." >> $LOG_FILE
echo "$(date '+%Y-%m-%d %H:%M:%S') - Service Successfully Logging..." >> $LOG_FILE
