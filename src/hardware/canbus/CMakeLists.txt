cmake_minimum_required(VERSION 3.5)
project(canbus)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(driverless_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_compile_definitions(QUTMS_CAN_VESC)
add_compile_definitions(QUTMS_CAN_VCU)
add_compile_definitions(QUTMS_CAN_BMU)
add_compile_definitions(QUTMS_CAN_RES)
add_compile_definitions(QUTMS_CAN_SW)

include_directories(include)
include_directories(${CMAKE_SOURCE_DIR}/../../hardware/QUTMS_Embedded_Common/Inc)

set (SOURCES
  src/can2ethernet_adapter.cpp
  src/tcp_client.cpp
  src/udp_client.cpp
  src/TritiumCAN.cpp
  src/SocketCAN.cpp
  ${CMAKE_SOURCE_DIR}/../../hardware/QUTMS_Embedded_Common/Src/CAN_VCU.c
  ${CMAKE_SOURCE_DIR}/../../hardware/QUTMS_Embedded_Common/Src/CAN_VESC.c
  ${CMAKE_SOURCE_DIR}/../../hardware/QUTMS_Embedded_Common/Src/CAN_BMU.c
)

SET_SOURCE_FILES_PROPERTIES(${SOURCES} PROPERTIES LANGUAGE CXX )

include_directories(${CMAKE_SOURCE_DIR}/../../hardware/CAN_Common/include/)

add_executable(canbus_translator_node src/node_canbus_translator.cpp ${SOURCES})
ament_target_dependencies(canbus_translator_node rclcpp driverless_msgs nav_msgs std_msgs)
target_include_directories(canbus_translator_node PUBLIC include PUBLIC ${CMAKE_SOURCE_DIR}/../../common/driverless_common/include)
target_include_directories(canbus_translator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(canbus_translator_socket_node src/node_canbus_translator_socket.cpp ${SOURCES})
ament_target_dependencies(canbus_translator_socket_node rclcpp driverless_msgs nav_msgs std_msgs)
target_include_directories(canbus_translator_socket_node PUBLIC include PUBLIC ${CMAKE_SOURCE_DIR}/../../common/driverless_common/include)
target_include_directories(canbus_translator_socket_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  canbus_translator_node canbus_translator_socket_node
  DESTINATION lib/${PROJECT_NAME}
)

# config folder
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
