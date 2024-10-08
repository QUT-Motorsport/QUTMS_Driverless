cmake_minimum_required(VERSION 3.8)
project(qutms_hw_interfaces CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow
    -Wpedantic -Wsign-conversion -Winit-self -Wredundant-decls )
endif()

option(BUILD_SHARED_LIBS "build shared libraries" ON)

# find dependencies
find_package(ackermann_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(driverless_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ros2_control_test_assets REQUIRED)

add_library(
  qutms_hw_interfaces
  SHARED
  src/qev_3d_topic_interface.cpp
)
target_include_directories(
  qutms_hw_interfaces
  PUBLIC
  include
)
ament_target_dependencies(
  qutms_hw_interfaces
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  ackermann_msgs
  driverless_msgs
)
# prevent pluginlib from using boost
target_compile_definitions(qutms_hw_interfaces PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

target_compile_features(qutms_hw_interfaces PUBLIC cxx_std_17)

pluginlib_export_plugin_description_file(
  hardware_interface qutms_hw_interfaces.xml)

install(
  TARGETS
  qutms_hw_interfaces
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)

  ament_add_gmock(test_qev_3d_topic_interface test/test_qev_3d_topic_interface.cpp)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  target_include_directories(test_qev_3d_topic_interface PRIVATE include)
  ament_target_dependencies(
    test_qev_3d_topic_interface
    hardware_interface
    pluginlib
    ros2_control_test_assets
    ackermann_msgs
    driverless_msgs
  )
endif()
ament_export_include_directories(
  include
)
ament_export_libraries(
  qutms_hw_interfaces
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ackermann_msgs
  driverless_msgs
)

ament_package()
