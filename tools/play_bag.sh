#!/bin/bash

source install/setup.bash

# use clock from bag as sim time and start paused
# BAG_CMD="ros2 bag play -s mcap /mnt/e/rosbag2_2024_07_11-05_28_15/ --clock -p"
# BAG_CMD="ros2 bag play -s mcap /mnt/e/rosbag2_2024_07_18-05_47_36/ --clock -p" # pushed aroud
# BAG_CMD="ros2 bag play -s mcap /mnt/e/rosbag2_2024_07_25-03_24_19/ --clock -p" # lidar on roll hoop test
# BAG_CMD="ros2 bag play -s mcap /mnt/e/rosbag2_2024_07_25-05_04_46/ --clock -p" # half lap
# BAG_CMD="ros2 bag play -s mcap /mnt/e/rosbag2_2024_07_25-05_18_42/ --clock -p" # other half of lap
# BAG_CMD="ros2 bag play -s mcap /mnt/e/rosbag2_2024_09_04-05_14_43 --clock -p" # 2.5 laps
BAG_CMD="ros2 bag play -s mcap /mnt/e/rosbag2_2024_09_24-02_11_49/ --clock -p" 
# BAG_CMD="ros2 bag play -s mcap /mnt/e/rosbag2_2024_09_24-02_16_55/ --clock -p" # ebs test run
# BAG_CMD="ros2 bag play -s mcap /mnt/e/rosbag2_2024_10_18-01_10_29/ --clock -p" # 9 laps, recording from 1st corner
# BAG_CMD="ros2 bag play -s mcap /mnt/e/rosbag2_2024_10_18-01_40_49 --clock -p" # 10 laps recording from 2nd lap, 1st corner
BAG_CMD="ros2 bag play -s mcap bags/rosbag2_2024_10_30-04_40_17 --clock -p" # 14 laps recording from 2nd lap, 1st corner
# BAG_CMD="ros2 bag play -s mcap /mnt/e/rosbag2_2024_11_01-19_02_10 --clock -p" # 14 laps recording from 2nd lap, 1st corner
# BAG_CMD="ros2 bag play -s mcap bags/trackdrive-2024-11-24-23-54-17 --clock -p" # 1 lap QR
# BAG_CMD="ros2 bag play -s mcap bags/ebs_test-2024-12-07-05-26-36 --clock -p" # comp ebs test
# BAG_CMD="ros2 bag play -s mcap bags/rosbag2_2024_11_12-03_35_31 --clock -p"

TOPICS=(
    # for mapping debugging
    /imu/odometry # needed for RL
    /vehicle/wheel_twist # needed for RL
    /lidar/objects
    /lidar/ground_points
    # /lidar/converted_2D_scan
    # /lidar/cone_detection
    # /debug_markers/lidar_markers
    # /system/av_state # needed for lap = 0 in mapping
    # /system/ros_state # needed for lap = 0 in mapping

    # raw data for visuals (nice to always have)
    /imu/nav_sat_fix
    # /vehicle/velocity
    /vehicle/steering_angle
    /imu/data
    # /control/driving_command

    # for fast lap debugging
    # /tf
    # /tf_static
    # /odometry/filtered
    # /slam/occupancy_grid
    # /slam/occupancy_grid_metadata
    # /slam/pose
    # /slam/graph_visualization
)

# remap these topics if you are running programs which output to the same topic names
REMAPS=(
    # /odometry/filtered:=/odometry/filtered_old
    # /sbg_translated/odometry:=/sbg_translated/odometry_old
    # /tf:=/tf_old
    # /control/driving_command:=/control/driving_command_old
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
