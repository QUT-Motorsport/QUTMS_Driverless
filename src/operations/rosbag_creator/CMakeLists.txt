cmake_minimum_required(VERSION 3.8)
project(rosbag_creator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(driverless_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_interfaces REQUIRED)

add_library(rosbag_creator_component SHARED
  src/rosbag2_recorder_component.cpp
)
target_include_directories(rosbag_creator_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(rosbag_creator_component PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  rosbag_creator_component
  "rclcpp"
  "rclcpp_components"
  "driverless_msgs"
  "rosbag2_transport"
  "rosbag2_cpp"
  "rosbag2_storage"
  "rosbag2_interfaces"
)

rclcpp_components_register_node(rosbag_creator_component
  PLUGIN "rosbag_creator::Rosbag2RecorderComponent"
  EXECUTABLE rosbag_creator_component_exec
)

add_executable(rosbag_creator_node src/rosbag_creator_node.cpp)
target_include_directories(rosbag_creator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(rosbag_creator_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  rosbag_creator_node
  "rclcpp"
  "driverless_msgs"
  "rosbag2_transport"
  "rosbag2_cpp"
  "rosbag2_storage"
  "rosbag2_interfaces"
)
target_link_libraries(rosbag_creator_node rosbag_creator_component)

install(TARGETS rosbag_creator_component rosbag_creator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
