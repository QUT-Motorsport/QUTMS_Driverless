cmake_minimum_required(VERSION 3.5)
project(vehicle_supervisor)

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
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_compile_definitions(QUTMS_CAN_VESC)
add_compile_definitions(QUTMS_CAN_VCU)
add_compile_definitions(QUTMS_CAN_DVL)
add_compile_definitions(QUTMS_CAN_RES)
add_compile_definitions(QUTMS_CAN_SW)
include_directories(${CMAKE_SOURCE_DIR}/../../hardware/QUTMS_Embedded_Common/Inc)
include_directories(${CMAKE_SOURCE_DIR}/../../hardware/CAN_Common/include/)

set (SOURCES
  ${CMAKE_SOURCE_DIR}/../../hardware/QUTMS_Embedded_Common/Src/CAN_VCU.c
  ${CMAKE_SOURCE_DIR}/../../hardware/QUTMS_Embedded_Common/Src/CAN_VESC.c
  ${CMAKE_SOURCE_DIR}/../../hardware/QUTMS_Embedded_Common/Src/CAN_DVL.c
  ${CMAKE_SOURCE_DIR}/../../hardware/QUTMS_Embedded_Common/Src/CAN_RES.c
  ${CMAKE_SOURCE_DIR}/../../hardware/QUTMS_Embedded_Common/Src/CAN_SW.c
  ${CMAKE_SOURCE_DIR}/../../hardware/CAN_Common/src/canopen.cpp
  ${CMAKE_SOURCE_DIR}/../../hardware/CAN_Common/src/can_interface.cpp
)

SET_SOURCE_FILES_PROPERTIES(${SOURCES} PROPERTIES LANGUAGE CXX )

add_executable(vehicle_supervisor_node src/node_supervisor.cpp ${SOURCES})
ament_target_dependencies(vehicle_supervisor_node rclcpp driverless_msgs ackermann_msgs std_msgs nav_msgs)
target_include_directories(vehicle_supervisor_node PUBLIC ${CMAKE_SOURCE_DIR}/../../common/driverless_common/include)
target_include_directories(vehicle_supervisor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS vehicle_supervisor_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
