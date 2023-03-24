#!/bin/bash

ros2 bag record /slam/car_pose /sbg/ekf_euler /sbg/ekf_nav /sbg/gps_pos /sbg/gps_vel /sbg/mag /sbg/mag_calibration /vision/cone_detection /lidar/cone_detection /slam/global_map /slam/local_map /imu/data /imu/nav_sat_fix /imu/pos_ecef /imu/velocity
