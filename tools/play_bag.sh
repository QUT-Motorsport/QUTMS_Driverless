#!/bin/bash

source install/setup.bash

# use clock from bag as sim time and start paused
BAG_CMD="ros2 bag play -s mcap /mnt/e/rosbag2_2024_05_21-05_25_56_mcap --clock -p"
# BAG_CMD="ros2 bag play /mnt/e/rosbag2_2024_05_15-05_27_12 --clock -p"

# list of topics we want to play
TOPICS=(
    # Lidar and boundary mapping tuning

    # /lidar/cone_detection
    # /slam/global_map
    # /scan
    # /velodyne_points
    # /zed2i/zed_node/left_raw/image_raw_color
    # /zed2i/zed_node/left_raw/camera_info
    # /zed2i/zed_node/right_raw/image_raw_color
    # /zed2i/zed_node/right_raw/camera_info
    # /tf
    # /tf_static
    # /system/as_status
    # /zed2i/zed_node/odom
    # /odometry/sbg_ekf
    # /imu/odometry

    # 3D lidar slam testing

    # /zed2i/zed_node/odom
    # /velodyne_points
    # /odometry/sbg_ekf
    # /tf_static
    # /imu/data
    # /scan

    # VD data

    # /vehicle/steering_angle
    # /slam/car_pose
    # /imu/data

    # RL and SLAM toolbox 
    /velodyne_points
    /lidar/cone_detection
    /vision/cone_detection
    # /scan
    /imu/odometry
    /imu/data


)

# remap these topics if you are running programs which output to the same topic names
REMAPS=(
    # /odometry/filtered:=/odometry/filtered_old
    # /sbg_translated/odometry:=/sbg_translated/odometry_old
    # /tf:=/tf_old
)

# for each topic append to the command
if [ ${#TOPICS[@]} -gt 0 ]; then
    BAG_CMD+=" --topics"
    for ((i=0; i<${#TOPICS[@]}; i++)); do
        BAG_CMD+=" ${TOPICS[$i]}"
    done
fi

# for each remap, create a remap string and append to the command
if [ ${#REMAPS[@]} -gt 0 ]; then
    BAG_CMD+=" --remap"
    for ((i=0; i<${#REMAPS[@]}; i++)); do
        BAG_CMD+=" ${REMAPS[$i]}"
    done
fi

# loop if arg is given
if [ "$1" == "loop" ]; then
    BAG_CMD+=" --loop"
fi

# run the command
echo $BAG_CMD
eval $BAG_CMD