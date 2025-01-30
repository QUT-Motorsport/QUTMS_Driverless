# ROS2 Control Package info



## ------- Files and what they do:

## Bringup:
# controller.yaml:
    Defines and configures all the controllers for hardware interfaces.
    - update rate for the controllers defined
    - controllers include bicycle_steering_controller, joint_state_broadcaster etc.
    - the parameters of the controllers are defined here
# controller_launch.py:
    Launch file starts and manages the robot. Script sets up controllers, state publishers and visualization tools (if there is any)
    - declares launch arguements
    - loads robots description (URDF/Xacro)
    - loads controller config (YAML)
    - launch ros2 nodes
## Robot Description:
# qev-3d.urdf.xacro: (in vehicle urdf)
    The big XACRO file linking all the URDF together
    - Defines alot of common parameters
# vehicle.urdf.xacro
    Descriptor file for the car itself, containing the actual 3D description of the car
    - including chassis type, wheel size and stuff
# qev3d_ros2_control.urdf.xacro:
    Defines robot's hardware interface and control setup
    - contains hardware plugins of the hardware interface
    - Defines the name and type of each joint
    - Defines command and state interfaces for each joint (such as steering and driving) as well as its parameters
    - note could have modes for real robot or sim (gazebo)
## Hardware interface:
# qev-3d_hardware_interface.hpp:
    Header file defining joint data structures and the ros2 hardware interface class setup
    Setup:
    - Jointvalue struct (stores joint state)
    - Joint struct (represent robot joint)
    - Qev3dHardwareInterface class and its methods
# qev-3d_hardware_interface.cpp:
    Source file defineing the hardware interface. Functions:
    - on_init()
    - on_configure()
    - on_activate()
    - on_deactivate()
    - read()
    - write

# CMakeLists.txt:
    Responsable for building and configuring the package
    -Tells CMake how to compile the source code

# package.xml:
    - Declares package metadata (name, version, license)
    - Specifies dependencies
    - Configures build system
    - Ensure ROS2 tools (like colcon) recognise and process the package
