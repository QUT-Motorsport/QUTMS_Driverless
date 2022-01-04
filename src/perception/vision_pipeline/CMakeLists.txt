cmake_minimum_required(VERSION 3.5)
project(zed_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# models folder
install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
)

# yolov5 folder
install(DIRECTORY
  yolov5
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
