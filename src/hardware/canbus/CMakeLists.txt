cmake_minimum_required(VERSION 3.5)
project(canbus)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(driverless_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_compile_definitions(QUTMS_CAN_VESC)
add_compile_definitions(QUTMS_CAN_VCU)
add_compile_definitions(QUTMS_CAN_RES)
add_compile_definitions(QUTMS_CAN_SW)
add_compile_definitions(QUTMS_CAN_EBS)

include_directories(
  include
  ${CMAKE_SOURCE_DIR}/../../hardware/QUTMS_Embedded_Common/Inc
  ${CMAKE_SOURCE_DIR}/../../hardware/CAN_Common/include/
  ${CMAKE_SOURCE_DIR}/../../common/driverless_common/include
)

set (SOURCES
  src/tcp_client.cpp
  src/udp_client.cpp
  src/TritiumCAN.cpp
  src/SocketCAN.cpp
  ${CMAKE_SOURCE_DIR}/../../hardware/QUTMS_Embedded_Common/Src/CAN_VCU.c
  ${CMAKE_SOURCE_DIR}/../../hardware/QUTMS_Embedded_Common/Src/CAN_VESC.c
  ${CMAKE_SOURCE_DIR}/../../hardware/CAN_Common/src/can_interface.cpp
)

SET_SOURCE_FILES_PROPERTIES(${SOURCES} PROPERTIES LANGUAGE CXX )

# ROS 2 components
add_library(canbus_translator_component SHARED src/component_canbus_translator.cpp ${SOURCES})
ament_target_dependencies(canbus_translator_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "nav_msgs"
  "driverless_msgs"
)
rclcpp_components_register_nodes(canbus_translator_component "canbus::CANTranslator")

# ROS 2 nodes
add_executable(socket_translator_node src/node_socket_translator.cpp)
target_link_libraries(socket_translator_node canbus_translator_component)
ament_target_dependencies(socket_translator_node
  "rclcpp"
)

add_executable(tritium_translator_node src/node_tritium_translator.cpp)
target_link_libraries(tritium_translator_node canbus_translator_component)
ament_target_dependencies(tritium_translator_node
  "rclcpp"
)

install(TARGETS
  canbus_translator_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  socket_translator_node tritium_translator_node
  DESTINATION lib/${PROJECT_NAME})

# config folder
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
