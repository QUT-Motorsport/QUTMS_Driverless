cmake_minimum_required(VERSION 3.8)
project(cpp_lidar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL 1.9 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

set (SOURCES
  src/node_detector.cpp
  PCL_DBSCAN/dbscan.cpp
)

add_executable(detector src/node_detector.cpp ${SOURCES})

target_include_directories(detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(detector PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(detector
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  PCL
)


install(TARGETS detector
  DESTINATION lib/${PROJECT_NAME})

ament_package()
