cmake_minimum_required(VERSION 3.8)
project(slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ackermann_msgs REQUIRED)

add_executable(node_ekf src/node_ekf.cpp)
target_include_directories(node_ekf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(node_ekf PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


install(TARGETS node_ekf
  DESTINATION lib/${PROJECT_NAME})

ament_package()
