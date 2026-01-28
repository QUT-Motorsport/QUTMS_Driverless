cmake_minimum_required(VERSION 3.16)
project(qev_controllers LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

# find dependencies
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
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  qev_controllers
  SHARED
  hardware/src/qev_hardware_interface.cpp
)
target_compile_features(qev_controllers PUBLIC cxx_std_17)
target_include_directories(qev_controllers PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/qev_controllers>
)
target_link_libraries(qev_controllers PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface qev_controllers.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/qev_controllers
)
install(
  DIRECTORY description/ros2_control description/urdf
  DESTINATION share/qev_controllers
)
install(
  DIRECTORY bringup/launch config
  DESTINATION share/qev_controllers
)
install(TARGETS qev_controllers
  EXPORT export_qev_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_qev_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
