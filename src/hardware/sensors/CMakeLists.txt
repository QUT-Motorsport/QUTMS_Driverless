cmake_minimum_required(VERSION 3.5)
project(sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# share folder
install(DIRECTORY
  launch config scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
