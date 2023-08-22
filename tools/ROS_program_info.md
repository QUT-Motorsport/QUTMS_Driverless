# ROS 2 Program Info

## Group: Common

### Executable Nodes

| Package | Node | <div style="width:30vw">Description</div> | Topics Subscribed | Topics Published |
| --- | --- | --- | --- | --- |
| `custom_viz` | `matrix` | TODO | `TODO` | `TODO` |
| `data_overlay` | `overlay` | Creates an overlay to display acceleration on a g-force circle and velocity. | `/vehicle/wheel_speed`, `/imu/data` | `/debug_imgs/data_overlay` |
| `driverless_common` | `display` | Displays top-down images of cone and map detections and planned paths. | `/vision/cone_detection`,   `/lidar/cone_detection`, `/slam/global_map`, `/slam/local_map`, `/control/driving_command`, `/planner/path` | `/debug_imgs/vision_det_img`, `/debug_imgs/lidar_det_img`, `/debug_imgs/slam_image`, `/debug_imgs/local_image`, `/markers/path_line` |
| `driverless_common` | `topic_to_csv` | Saves ROS topic data to CSV files. | Any specified | n/a |

### Launch Files

| Package | Launch File | <div style="width:30vw">Description</div> |
| --- | --- | --- |
| `driverless_common` | `display.launch.py` | Launches the `display` node with `rosboard_node` (submodule package) for web server and visuals. |
| `vehicle_urdf` | `robot_description.launch.py` | Launches the `robot_state_publisher` node and `joint_state_publisher` node with the URDF file for QEV-3D. |

## Group: Control

### Executable Nodes

| Package | Node | <div style="width:30vw">Description</div> | Topics Subscribed | Topics Published |
| --- | --- | --- | --- | --- |
| `controllers` | `sine` | Steers in a sine wave from lock to lock for the inspection. | n/a | `/control/driving_command`, `/control/accel_command` |
| `controllers` | `reactive_control` | Follows a mid point between the furthest pair of local cones seen. If only one colour cone can be seen, the controller will output full-lock. If no cones are seen, the controller halts. | `/system/as_status`, `/lidar/cone_detection`, `/slam/local_map` | `/control/driving_command` |
| `controllers` | `point_fitting` | Fits a quadratic curve to minimise the error between the local cone boundaries. | `/slam/local_map`,  | `/control/driving_command`, `/debug_imgs/control_img` |
| `controllers` | `point_fitting2` | Fits a steering curve to minimise the error between the local cone boundaries. | `/lidar/cone_detection` | `/control/driving_command`, `/debug_imgs/control_img` |
| `controllers` | `vector_spline` | Creates vectors between local cone boundaries and follows a mid spline perpendicular to them. | `/slam/local_map` | `/control/driving_command` |
| `path_follower` | `pure_pursuit` | Follows a path by approaching a receding waypoint. | `/system/as_status`, `/planner/path`, `/slam/car_pose` | `/control/driving_command`, `debug_imgs/pursuit_img` |
| `path_follower` | `pure_pursuit_kdtree` | Follows a path by approaching a receding waypoint. Waypoint is searched in a KDTree. | `/planner/path`, `/slam/car_pose` | `/control/driving_command`, `debug_imgs/pursuit_img` |
| `pure_pursuit_cpp` | `pure_pursuit_cpp` | Follows a path by approaching a receding waypoint. Written in C++. | `/planner/path`, `/slam/car_pose` | `/control/driving_command` |
| `steering_testing` | `random` | Generates steering commands randomly between full lock. | n/a | `/control/driving_command` |
| `steering_testing` | `step_response` | Generates a steering command to one specified angle and then its opposite. | n/a | `/control/driving_command` |
| `steering_testing` | `step_response_calibration` | Records and graphs stepper motor encoder increments as steering commands are generated from lock to lock and back again. | `/vehicle/steering_reading`, `/vehicle/encoder_reading` | `/control/driving_command`, `/debug_imgs/model_calibration_image` |
`yaw_controller` | `yaw_controller` | Controls the yaw of the vehicle to a specified angle. | `/imu/velocity`, `/imu/data` `/system/as_status` | `/control/driving_command` |

## Group: Hardware

### Executable Nodes

| Package | Node | <div style="width:30vw">Description</div> | Topics Subscribed | Topics Published |
| --- | --- | --- | --- | --- |
| `canbus` | `canbus_translator_node` | Reads CAN messages from the vehicle and publishes them as various sensor topics. | n/a | `/vehicle/steering_angle`, `/vehicle/velocity`, `/vehicle/wheel_odom`, `/vehicle/bmu_status` |
| `sbg_testing` | `sbg_performance` | Plots SBG position tracking data for analysis. | `/imu/nav_sat_fix`, `/imu/velocity`, `/sbg/ekf_nav` | n/a |
| `sbg_testing` | `sbg_debug` | Publishes all SBG data as plots. | Too many | Too many |
| `steering_actuator` | `steering_actuator_node` | Relates a target steering angle to a number of rotations. | `/control/driving_command`, `/vehicle/steering_angle`, `/can/canbus_rosbound` | `/can/canbus_carbound`, `/vehicle/encoder_reading` |
| `steering_actuator` | `steering_actuator_node_old` | Controls the steering actuator to a specified angle based on the steering angle. | `/control/driving_command`, `/vehicle/steering_angle`, `/can/canbus_rosbound` | `/can/canbus_carbound`, `/vehicle/encoder_reading` |
| `vehicle_supervisor` | `vehicle_supervisor_node` | Supervises the vehicle state. Reads CAN messages and converts them to specific datatypes. | `/can/canbus_rosbound`, `/vehicle/accel_command`, `/system/shutdown` | `/can/canbus_carbound`, `/system/as_status`, `/system/reset`, `/data_logger/drivingDynamics1`, `/data_logger/systemStatus`, `/vehicle/res_status`, `/vehicle/motor_rpm`, `/vehicle/wheel_speed`, `/vehicle/steering_angle`, `/vehicle/wheel_odom`  |
| `vehicle_supervisor` | `vehicle_supervisor_node` | Supervises the vehicle and publishes the vehicle status. | `/can/canbus_rosbound`, `/vehicle/steering_angle`, `/vehicle/velocity`, `/vehicle/wheel_odom`, `/vehicle/accel_command`, `/system/laps_completed`, `/system/shutdown` | `/can/canbus_carbound`, `/system/as_status`, `/system/res_status`, `/system/reset`, `/data_logger/drivingDynamics1`, `/data_logger/systemStatus` |
| `velocity_controller` | `velocity_controller_node` | Controls the velocity of the vehicle to a specified velocity. | `/vehicle/motor_rpm`, `/vehicle/driving_command`, `/system/as_status` | `/control/accel_command` |

### Launch Files

| Package | Launch File | <div style="width:30vw">Description</div> |
| --- | --- | --- |
| `steering_actuator` | `machine.launch.py` | Launches the `steering_actuator_node` with `steering.yaml` parameters. |
| `sensors` | `sbg_device_mag_calibration.launch.py` | Launches the SBG magnetometer calibration with our parameters file. |
| `sensors` | `sbg_device.launch.py` | Launches the SBG driver with our parameters file. |
| `sensors` | `vlp32.launch.py` | Launches the VLP32 lidar driver, pointcloud, and laserscan converters with our parameters file. |
| `sensors` | `zed_camera.launch.py` | Launches the ZED 2i camera with our parameters file. |

## Group: Machines

### Launch Files
| Package | Launch File | <div style="width:30vw">Description</div> |
| --- | --- | --- |
| `jetson_machine` | `machine.launch.py` | Launches the camera driver and `vision_pipeline` detector node |
| `roscube_machine` | `machine.launch.py` | Launches sensor drivers, vehicle supervisor, can translator, and vehicle operations nodes |

## Group: Navigation

### Executable Nodes

| Package | Node | <div style="width:30vw">Description</div> | Topics Subscribed | Topics Published |
| --- | --- | --- | --- | --- |
| `planners` | `ordered_mid_spline` | Orders the track boundaries, interpolates between, and creates a mid spline through the center of the track. | `/slam/global_map`, `/system/as_status` | `/planner/path`, `planner/spline_path`, `/planner/interpolated_map` |
| `py_slam` | `imu_slam` | EKF SLAM implementation using IMU for localisation. Creates a map of new cones as the car navigates the discovery lap. | `/imu/velocity`, `/lidar/cone_detection`, `/vision/cone_detection`, `/system/reset` | `/slam/global_map`, `/slam/car_pose`, `/slam/local_map` |
| `py_slam` | `wss_slam` | EKF SLAM implementation using wheel speeds for localisation. | `/imu/velocity`, `/vehicle/wheel_speed`, `/lidar/cone_detection`, `/vision/cone_detection`, `/system/reset` | `/slam/global_map`, `/slam/car_pose`, `/slam/local_map` |
| `py_slam` | `sbg_slam` | EKF SLAM implementation using SBG GPS for localisation. | `/sbg/gps_pos`, `/sbg/ekf_euler`, `/lidar/cone_detection`, `/vision/cone_detection`, `/system/reset` | `/slam/global_map`, `/slam/car_pose`, `/slam/local_map` |
| `py_slam` | `odom_slam` | EKF SLAM implementation using a published odom->base transform for localisation. | `/tf`, `/lidar/cone_detection`, `/vision/cone_detection`, `/system/as_status` | `/slam/global_map`, `/slam/car_pose`, `/slam/local_map` |
| `py_slam` | `track_to_csv` | Converts a discovered global map to a csv file. | `/slam/global_map` | n/a |
| `slam` | `node_ekf_slam` | EKF SLAM in C++ using IMU for localisation. | `velocity`, `cone_detection` | `/slam/track`, `/slam/pose` |

## Group: Operations

### Executable Nodes

| Package | Node | <div style="width:30vw">Description</div> | Topics Subscribed | Topics Published |
| --- | --- | --- | --- | --- |
| `mission_controller` | `mission_launcher` | Runs ROS 2 launch script with the specified mission. | `/system/as_status` | n/a |
| `mission_controller` | `inspection_handler` | Oversees the inspection mission starting and finishing. | `/system/reset` | `/system/shutdown` |
| `mission_controller` | `trackdrive_handler` | Oversees the trackdrive mission through the 10 laps. | `/system/as_status`, `/slam/car_pose` | `/system/shutdown`, `/system/laps_completed` |
| `terminal_control` | `controller` | Terminal GUI interface for vehicle control and mission selection. | `/system/as_status` | `/system/mission_select`, `/system/reset`, `system/r2d`, `system/laps_completed` |

### Launch Files

| Package | Launch File | <div style="width:30vw">Description</div> |
| --- | --- | --- |
| `mission_controller` | `inspection.launch.py` | Launches the inspection mission and handler - simple sine control. |
| `mission_controller` | `trackdrive.launch.py` | Launches the trackdrive mission and handler - discovery lap control, mapping, planning, following. |
| `mission_controller` | `ebs_test.launch.py` | Launches the EBS test mission and handler - reactive control. |
| `mission_controller` | `manual_driving.launch.py` | Launches *nothing* |

## Group: Perception

### Executable Nodes

| Package | Node | <div style="width:30vw">Description</div> | Topics Subscribed | Topics Published |
| --- | --- | --- | --- | --- |
| `vision_pipeline` | `detector` | Detects cone locations using ZED camera. Has PyTorch, TensorRT, HSV implementations | `/zed2i/zed_node/rgb/image_rect_color`, `/zed2i/zed_node/rgb/camera_info`, `/zed2i/zed_node/depth/depth_registered` | `/vision/cone_detection`, `/debug_imgs/vision_bbs_img`, `/debug_imgs/vision_depth_img` |
| `lidar_pipeline` | `detector` | Detects cone locations using LiDAR. | `/velodyne_points` | `/lidar/cone_detection` |
| `cpp_lidar` | `detector` | Experimental node in C++ to detect aspects of a pointcloud. | `/velodyne_points` | `/velodyne/ground`, `/velodyne/non_ground`, `/velodyne/clusters` |
| `hsv_thresholder` | `gui` | Slider GUI for tuning HSV thresholds. | n/a | `/hsv_thresholder/threshold` |
| `hsv_thresholder` | `thresholder` | Display for HSV threshold tuning. | `/zed2/zed_node/rgb/image_rect_color`, `/hsv_thresholder/threshold` | n/a |
