# ROS 2 Program Info

## Group: Common 

### Nodes with packages

| Package | Node | Description | Topics Subscribed | Topics Published |
| --- | --- | --- | --- | --- |
| `custom_viz` | `matrix` | TODO | `TODO` | `TODO` |
| `data_overlay` | `overlay` | Creates an overlay to display acceleration on a g-force circle and velocity. | `/vehicle/wheel_speed`, `/imu/data` | `/debug_imgs/data_overlay` |
| `driverless_common` | `display` | Displays top-down images of cone and map detections and planned paths. | `/vision/cone_detection`,   `/lidar/cone_detection`, `/slam/global_map`, `/slam/local_map`, `/control/driving_command`, `/planner/path` | `/debug_imgs/vision_det_img`, `/debug_imgs/lidar_det_img`, `/debug_imgs/slam_image`, `/debug_imgs/local_image`, `/markers/path_line` |
| `driverless_common` | `topic_to_csv` | Saves ROS topic data to CSV files. | Any specified | n/a |

### Launch Files with packages

| Package | Launch File | Description |
| --- | --- | --- |
| `driverless_common` | `display.launch.py` | Launches the `display` node with `rosboard_node` (submodule package) for web server and visuals. |
| `vehicle_urdf` | `robot_description.launch.py` | Launches the `robot_state_publisher` node and `joint_state_publisher` node with the URDF file for QEV-3D. |

## Group: Control

### Nodes with packages

| Package | Node | Description | Topics Subscribed | Topics Published |
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