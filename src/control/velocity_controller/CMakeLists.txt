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
find_package(rclcpp_components REQUIRED)

include_directories(
  include
  ${CMAKE_SOURCE_DIR}/../../common/driverless_common/include
)

SET_SOURCE_FILES_PROPERTIES(${SOURCES} PROPERTIES LANGUAGE CXX )

# ROS 2 components
add_library(velocity_controller_component SHARED src/component_velocity_controller.cpp)
ament_target_dependencies(velocity_controller_component
  "rclcpp"
  "rclcpp_components"
  "rcl_interfaces"
  "std_msgs"
  "nav_msgs"
  "ackermann_msgs"
  "driverless_msgs"
)
rclcpp_components_register_nodes(velocity_controller_component "velocity_controller::VelocityController")

# ROS 2 nodes
add_executable(velocity_controller_node src/node_velocity_controller.cpp)
target_link_libraries(velocity_controller_node velocity_controller_component)
ament_target_dependencies(velocity_controller_node
  "rclcpp"
)

install(TARGETS
  velocity_controller_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS velocity_controller_node
  DESTINATION lib/${PROJECT_NAME})

# config folder
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
