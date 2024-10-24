cmake_minimum_required(VERSION 3.8)
project(QUTMS_Embedded_Common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# launch folder
install(DIRECTORY
  QUTMS_Embedded_Common
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
