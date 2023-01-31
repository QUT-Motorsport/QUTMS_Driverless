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
find_package(rclcpp REQUIRED)

include_directories(include)
add_compile_definitions(QUTMS_CAN_VCU)
include_directories(${CMAKE_SOURCE_DIR}/../../hardware/QUTMS_Embedded_Common/Inc)
include_directories(${CMAKE_SOURCE_DIR}/../../hardware/CAN_Common/include/)

set (SOURCES
  ${CMAKE_SOURCE_DIR}/../../hardware/QUTMS_Embedded_Common/Src/CAN_VCU.c
  ${CMAKE_SOURCE_DIR}/../../hardware/CAN_Common/src/canopen.cpp
  ${CMAKE_SOURCE_DIR}/../../hardware/CAN_Common/src/can_interface.cpp
)

SET_SOURCE_FILES_PROPERTIES(${SOURCES} PROPERTIES LANGUAGE CXX )

add_executable(steering src/node_steering_actuator.cpp ${SOURCES})
ament_target_dependencies(steering rclcpp ackermann_msgs driverless_msgs)

target_include_directories(steering PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS steering
  DESTINATION lib/${PROJECT_NAME})

# config folder
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
