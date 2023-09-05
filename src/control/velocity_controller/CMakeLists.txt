cmake_minimum_required(VERSION 3.5)
project(velocity_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(driverless_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

SET_SOURCE_FILES_PROPERTIES(${SOURCES} PROPERTIES LANGUAGE CXX )

add_executable(velocity_controller_node src/node_velocity_controller.cpp ${SOURCES})
target_include_directories(velocity_controller_node PUBLIC ${CMAKE_SOURCE_DIR}/../../common/driverless_common/include)
target_include_directories(velocity_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(velocity_controller_node
  rclcpp
  rcl_interfaces
  nav_msgs
  ackermann_msgs
  driverless_msgs)

install(TARGETS velocity_controller_node
  DESTINATION lib/${PROJECT_NAME})

# config folder
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
