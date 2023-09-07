cmake_minimum_required(VERSION 3.5)
project(sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sbg_driver REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(sbg_translator_node src/node_sbg_translator.cpp ${SOURCES})
ament_target_dependencies(sbg_translator_node
  rclcpp
  sbg_driver
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  sensor_msgs
)

target_include_directories(sbg_translator_node PUBLIC
  ${CMAKE_SOURCE_DIR}/../../common/driverless_common/include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  TARGETS sbg_translator_node
  DESTINATION lib/${PROJECT_NAME}
)

# linked folders
install(
  DIRECTORY launch config scripts camera_files
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
