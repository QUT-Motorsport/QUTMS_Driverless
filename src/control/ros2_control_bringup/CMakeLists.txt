cmake_minimum_required(VERSION 3.5)
project(ros2_control_bringup)

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch, config and urdf directories
install(DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
