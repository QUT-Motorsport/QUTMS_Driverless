cmake_minimum_required(VERSION 3.5)
project(car_status)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(driverless_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_compile_definitions(QUTMS_CAN_BMU)

include_directories(include)
include_directories(${CMAKE_SOURCE_DIR}/../../hardware/CAN_Common/include/)
include_directories(${CMAKE_SOURCE_DIR}/../../hardware/QUTMS_Embedded_Common/Inc)

set (SOURCES
  ${CMAKE_SOURCE_DIR}/../../hardware/QUTMS_Embedded_Common/Src/CAN_BMU.c
  ${CMAKE_SOURCE_DIR}/../../hardware/CAN_Common/src/canopen.cpp
  ${CMAKE_SOURCE_DIR}/../../hardware/CAN_Common/src/can_interface.cpp
)

SET_SOURCE_FILES_PROPERTIES(${SOURCES} PROPERTIES LANGUAGE CXX )

add_executable(car_status_node src/node_car_status.cpp ${SOURCES})
ament_target_dependencies(car_status_node rclcpp driverless_msgs)

target_include_directories(car_status_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS car_status_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
