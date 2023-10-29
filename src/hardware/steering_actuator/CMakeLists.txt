cmake_minimum_required(VERSION 3.5)
project(steering_actuator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(ackermann_msgs REQUIRED)
find_package(driverless_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)
include_directories(${CMAKE_SOURCE_DIR}/../../hardware/CAN_Common/include/)
include_directories(${CMAKE_SOURCE_DIR}/../../common/driverless_common/include)

set (SOURCES
  ${CMAKE_SOURCE_DIR}/../../hardware/CAN_Common/src/canopen.cpp
  ${CMAKE_SOURCE_DIR}/../../hardware/CAN_Common/src/can_interface.cpp
)

add_executable(steering_actuator_node src/node_steering_actuator.cpp ${SOURCES})
add_executable(steering_actuator_test_node src/node_steering_actuator_test.cpp ${SOURCES})

ament_target_dependencies(steering_actuator_node rclcpp ackermann_msgs driverless_msgs std_msgs)
ament_target_dependencies(steering_actuator_test_node rclcpp ackermann_msgs driverless_msgs std_msgs)

target_include_directories(steering_actuator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(steering_actuator_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

  install(TARGETS steering_actuator_node steering_actuator_test_node
  DESTINATION lib/${PROJECT_NAME})

# config folder
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
