(
    source /opt/ros/noetic/setup.bash
    roscore
) &

sleep 15

(
    source $HOME/Formula-Student-Driverless-Simulator/ros/devel/setup.bash
    roslaunch fsds_ros_bridge fsds_ros_bridge.launch host:=$SIM_HOST manual_mode:=$MANUAL_MODE
) &

(
    source $HOME/bridge_ws/install/setup.bash
    ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
) &

wait