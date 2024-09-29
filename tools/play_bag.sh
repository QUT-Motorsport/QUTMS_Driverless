#!/bin/bash

source install/setup.bash

# use clock from bag as sim time and start paused
# BAG_CMD="ros2 bag play -s mcap /mnt/e/rosbag2_2024_07_11-05_28_15/ --clock -p"
# BAG_CMD="ros2 bag play -s mcap /mnt/e/rosbag2_2024_07_18-05_47_36/ --clock -p" # pushed aroud
# BAG_CMD="ros2 bag play -s mcap /mnt/e/rosbag2_2024_07_25-03_24_19/ --clock -p" # lidar on roll hoop test
# BAG_CMD="ros2 bag play -s mcap /mnt/e/rosbag2_2024_07_25-05_04_46/ --clock -p" # half lap
# BAG_CMD="ros2 bag play -s mcap /mnt/e/rosbag2_2024_07_25-05_18_42/ --clock -p" # other half of lap
# BAG_CMD="ros2 bag play -s mcap /mnt/e/rosbag2_2024_09_04-05_14_43/ --clock -p" # 2.5 laps
BAG_CMD="ros2 bag play -s mcap /mnt/e/rosbag2_2024_09_24-06_08_59/ --clock -p" # 1 lap, loop close crashing

REMAPPINGS=(
    # /tf:=/tf_old
    # /tf_static:=/tf_static_old
    # /slam/occupancy_grid:=/slam/occupancy_grid_old
    # /slam/car_pose:=/slam/car_pose_old
    # /odom/filtered:=/odom/filtered_old
)

TOPICS=(
    /imu/odometry # needed for RL
    /vehicle/wheel_twist # needed for RL
    # /velodyne_points
    /lidar/converted_2D_scan
    /lidar/cone_detection
    # /debug_markers/lidar_markers
    /system/as_status # needed for lap = 0 in mapping
    # /imu/nav_sat_fix # for visual
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
