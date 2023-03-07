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

include_directories(include)

set (SOURCES
  src/can2ethernet_adapter.cpp
  src/tcp_client.cpp
  src/udp_client.cpp
  src/TritiumCAN.cpp
)

add_executable(canbus_translator_node src/node_canbus.cpp ${SOURCES})
ament_target_dependencies(canbus_translator_node rclcpp driverless_msgs)

target_include_directories(canbus_translator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  canbus_translator_node
  DESTINATION lib/${PROJECT_NAME}
)

# config folder
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
