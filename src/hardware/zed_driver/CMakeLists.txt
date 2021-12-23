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

# launch folder
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# config folder
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
