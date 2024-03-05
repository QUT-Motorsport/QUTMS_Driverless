cmake_minimum_required(VERSION 3.5)
project(vehicle_supervisor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(driverless_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

add_compile_definitions(QUTMS_CAN_VCU)
add_compile_definitions(QUTMS_CAN_DVL)
add_compile_definitions(QUTMS_CAN_RES)
add_compile_definitions(QUTMS_CAN_EBS)
add_compile_definitions(QUTMS_CAN_SW)

include_directories(
  include
  ${CMAKE_SOURCE_DIR}/../../hardware/QUTMS_Embedded_Common/Inc
  ${CMAKE_SOURCE_DIR}/../../hardware/CAN_Common/include
  ${CMAKE_SOURCE_DIR}/../../common/driverless_common/include
)

set (SOURCES
  ${CMAKE_SOURCE_DIR}/../../hardware/QUTMS_Embedded_Common/Src/CAN_VCU.c
  ${CMAKE_SOURCE_DIR}/../../hardware/QUTMS_Embedded_Common/Src/CAN_DVL.c
  ${CMAKE_SOURCE_DIR}/../../hardware/QUTMS_Embedded_Common/Src/CAN_EBS_CTRL.c
  ${CMAKE_SOURCE_DIR}/../../hardware/QUTMS_Embedded_Common/Src/CAN_RES.c
  ${CMAKE_SOURCE_DIR}/../../hardware/QUTMS_Embedded_Common/Src/CAN_SW.c
  ${CMAKE_SOURCE_DIR}/../../hardware/CAN_Common/src/canopen.cpp
  ${CMAKE_SOURCE_DIR}/../../hardware/CAN_Common/src/can_interface.cpp
)

SET_SOURCE_FILES_PROPERTIES(${SOURCES} PROPERTIES LANGUAGE CXX )

# ROS 2 components
add_library(supervisor_component SHARED src/component_supervisor.cpp ${SOURCES})
ament_target_dependencies(supervisor_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "nav_msgs"
  "driverless_msgs"
  "ackermann_msgs"
)
rclcpp_components_register_nodes(supervisor_component "vehicle_supervisor::ASSupervisor")

add_library(supervisor_launch_component SHARED src/component_supervisor_launch.cpp src/component_supervisor.cpp ${SOURCES})
ament_target_dependencies(supervisor_launch_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "nav_msgs"
  "driverless_msgs"
  "ackermann_msgs")
rclcpp_components_register_nodes(supervisor_launch_component "vehicle_supervisor::ASSupervisorLaunch")

# ROS 2 nodes
add_executable(supervisor_node src/node_supervisor.cpp)
target_link_libraries(supervisor_node supervisor_component)
ament_target_dependencies(supervisor_node
  "rclcpp"
)

add_executable(supervisor_launch_node src/node_supervisor_launch.cpp)
target_link_libraries(supervisor_launch_node supervisor_launch_component supervisor_component)
ament_target_dependencies(supervisor_launch_node
  "rclcpp"
)

install(TARGETS
  supervisor_component supervisor_launch_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  supervisor_node supervisor_launch_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
