cmake_minimum_required(VERSION 3.5)
project(control_package)

# Set standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(fs_msgs REQUIRED)
find_package(qutms_msgs REQUIRED)

message(STATUS "Found OpenCV version ${OpenCV_VERSION}")

add_executable(control_movement src/control_movement.cpp)

ament_target_dependencies(control_movement OpenCV rclcpp sensor_msgs cv_bridge fs_msgs qutms_msgs)

install(TARGETS control_movement
  DESTINATION lib/${PROJECT_NAME})

ament_package()
