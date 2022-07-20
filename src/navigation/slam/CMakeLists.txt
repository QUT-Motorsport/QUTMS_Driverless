cmake_minimum_required(VERSION 3.8)
project(slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(driverless_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(node_ekf_slam src/ekf_slam/node_ekf_slam.cpp)
target_include_directories(node_ekf_slam PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(node_ekf_slam PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(node_ekf_slam
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  ackermann_msgs
  driverless_msgs
  message_filters
  visualization_msgs
)


install(TARGETS node_ekf_slam
  DESTINATION lib/${PROJECT_NAME})

ament_package()
