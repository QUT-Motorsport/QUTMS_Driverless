cmake_minimum_required(VERSION 3.5)
project(can_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(ackermann_msgs REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include/can_driver)

set (SOURCES
  src/can2ethernet_adapter.cpp
  src/tcp_client.cpp
  src/canopen.cpp
  src/steering_control.cpp
)

add_executable(can_driver src/can_driver.cpp ${SOURCES})
ament_target_dependencies(can_driver ackermann_msgs)

target_include_directories(can_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(steering src/node_steering.cpp ${SOURCES})
ament_target_dependencies(steering rclcpp ackermann_msgs)

target_include_directories(steering PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS can_driver steering
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
