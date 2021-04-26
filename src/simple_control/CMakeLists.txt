cmake_minimum_required(VERSION 3.5)
project(simple_control)

# Set standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

message(STATUS "Found OpenCV version ${OpenCV_VERSION}")

add_executable(main src/main.cpp)
ament_target_dependencies(main OpenCV rclcpp sensor_msgs cv_bridge)

install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME})

ament_package()
