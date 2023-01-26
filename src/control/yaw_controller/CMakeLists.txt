cmake_minimum_required(VERSION 3.5)
project(yaw_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(driverless_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

SET_SOURCE_FILES_PROPERTIES(${SOURCES} PROPERTIES LANGUAGE CXX )

add_executable(yaw_controller src/node_yaw_controller.cpp ${SOURCES})
ament_target_dependencies(yaw_controller
    rclcpp
    driverless_msgs
    ackermann_msgs
    sensor_msgs
    tf2
)

target_include_directories(yaw_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS yaw_controller
  DESTINATION lib/${PROJECT_NAME})

# config folder
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
