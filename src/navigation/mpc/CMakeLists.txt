cmake_minimum_required(VERSION 3.8)
project(mpc)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(driverless_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(fs_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# set (SOURCES
#   src/mpc_transforms.cpp
# )


add_executable(mpc_transforms src/mpc_transforms.cpp)
target_include_directories(mpc_transforms PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mpc_transforms PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(mpc_transforms
rclcpp
std_msgs
geometry_msgs
driverless_msgs
nav_msgs
fs_msgs
builtin_interfaces)

install(TARGETS mpc_transforms
  DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
