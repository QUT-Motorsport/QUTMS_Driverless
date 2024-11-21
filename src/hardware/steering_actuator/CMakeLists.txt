cmake_minimum_required(VERSION 3.5)
project(steering_actuator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(driverless_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(
  include
  ${CMAKE_SOURCE_DIR}/../../common/QUTMS_Embedded_Common/include/
  ${CMAKE_SOURCE_DIR}/../../common/driverless_common/include
)

set (SOURCES
  ${CMAKE_SOURCE_DIR}/../../common/QUTMS_Embedded_Common/src/canopen.cpp
  ${CMAKE_SOURCE_DIR}/../../common/QUTMS_Embedded_Common/src/can_interface.cpp
)

# ROS 2 components
add_library(steering_actuator_component SHARED src/component_steering_actuator.cpp ${SOURCES})
ament_target_dependencies(steering_actuator_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "driverless_msgs"
  "ackermann_msgs"
  "control_toolbox"
)
rclcpp_components_register_nodes(steering_actuator_component "steering_actuator::SteeringActuator")

# ROS 2 nodes
add_executable(steering_actuator_node src/node_steering_actuator.cpp)
target_link_libraries(steering_actuator_node steering_actuator_component)
ament_target_dependencies(steering_actuator_node
  "rclcpp"
)

install(TARGETS
  steering_actuator_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  steering_actuator_node
  DESTINATION lib/${PROJECT_NAME})

# config folder
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
