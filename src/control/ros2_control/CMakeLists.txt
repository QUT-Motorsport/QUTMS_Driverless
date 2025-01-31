# Minimum CMake version required
cmake_minimum_required(VERSION 3.16)

# Project name and language
project(ros2_control LANGUAGES CXX)

# Add compile options for GCC and Clang
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# Set the same behavior for Windows as it is on Linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required.")
endif()

# Find additional dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Compile the library
add_library(
  ros2_control
  SHARED
  src/my_controller.cpp  # Rename to your own source files
  src/my_hardware.cpp    # Rename to your own source files
)
target_compile_features(ros2_control PUBLIC cxx_std_17)
target_include_directories(ros2_control PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/ros2_control>
)
ament_target_dependencies(
  ros2_control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ros2_control.xml)  # Rename to your own plugin description file

# Install include directory
install(
  DIRECTORY include/
  DESTINATION include/ros2_control
)

# Install additional directories
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/ros2_control
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/ros2_control
)

# Install the library
install(TARGETS ros2_control
  EXPORT export_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Uncomment and modify the following lines to add tests
# if(BUILD_TESTING)
#   find_package(ament_cmake_pytest REQUIRED)
#   ament_add_pytest_test(example_11_urdf_xacro test/test_urdf_xacro.py)  # Rename to your own test files
#   ament_add_pytest_test(view_example_11_launch test/test_view_robot_launch.py)  # Rename to your own test files
#   ament_add_pytest_test(run_example_11_launch test/test_carlikebot_launch.py)  # Rename to your own test files
#   ament_add_pytest_test(run_example_11_launch_remapped test/test_carlikebot_launch_remapped.py)  # Rename to your own test files
# endif()

# Export targets and dependencies
ament_export_targets(export_ros2_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
