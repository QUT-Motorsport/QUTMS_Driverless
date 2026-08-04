cmake_minimum_required(VERSION 3.5)
project(ros2_control_bringup)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Build C++ sine test node
add_executable(sine_node src/sine_node.cpp)
ament_target_dependencies(sine_node rclcpp std_msgs)

# Install executable and directories
install(TARGETS sine_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Register the launch testing integration test
  add_launch_test(test/test_mock_integration.py)
endif()

ament_package()
