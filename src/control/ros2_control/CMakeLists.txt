# Minimum CMake version required
cmake_minimum_required(VERSION 3.16)
find_package(vehicle_urdf REQUIRED)
# Define the project name macro
set(PROJECT_NAME qev3d_ros2_control)
# Project name and language
project(${PROJECT_NAME})

# Add compile options for GCC and Clang
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# Set the same behavior for Windows as it is on Linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Define the dependencies macro
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

# Find additional dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Compile the library
add_library(
  ${PROJECT_NAME}
  SHARED
  hardware_interface/src/qev3d_hardware_interface.cpp    # Rename to your own source files
)

# Set include directories for the library
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hardware_interface/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Specify dependencies for the library
ament_target_dependencies(
  ${PROJECT_NAME} PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface qev3d_hardware_plugin.xml)

# Install include directory
install(
  DIRECTORY hardware_interface/include/
  DESTINATION include/${PROJECT_NAME}
)

# Install the description directory
install(DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/${PROJECT_NAME}
)

# Install the library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)


install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/${PROJECT_NAME}

)

install(FILES
qev3d_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)
# Uncomment and modify the following lines to add tests
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(example_11_urdf_xacro test/test_qev3d_ros2_control_launch.py)  # Rename to your own test files
endif()

# Export include directories
ament_export_include_directories(
  hardware_interface/include
)
# Export libraries
ament_export_libraries(${PROJECT_NAME})

# Export targets (e.g. libraries, executables, etc.), used when you want to export both library and executables
# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Export dependencies
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
