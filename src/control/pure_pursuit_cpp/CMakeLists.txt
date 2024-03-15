cmake_minimum_required(VERSION 3.5)
project(pure_pursuit_cpp)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif ()

# find dependencies
set(Dependencies rclcpp rclcpp_lifecycle lifecycle_msgs ackermann_msgs driverless_msgs nav_msgs tf2 tf2_ros)

foreach(Dependency ${Dependencies})
    find_package(${Dependency} REQUIRED)
endforeach()

set(SOURCES src/pure_pursuit.cpp src/node_pure_pursuit.cpp src/lifecycle_pure_pursuit.cpp)

# Library for common functions
add_library(pure_pursuit ${SOURCES})
ament_target_dependencies(pure_pursuit ${Dependencies})
target_include_directories(pure_pursuit PUBLIC
        ${CMAKE_SOURCE_DIR}/../../common/driverless_common/include)
target_include_directories(pure_pursuit PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
install(TARGETS pure_pursuit
        DESTINATION lib/${PROJECT_NAME})

# Executable for node_pure_pursuit
add_executable(pure_pursuit_node src/node_pure_pursuit.cpp)
ament_target_dependencies(pure_pursuit_node ${Dependencies})
target_include_directories(pure_pursuit_node PUBLIC
        ${CMAKE_SOURCE_DIR}/../../common/driverless_common/include)
target_include_directories(pure_pursuit_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(pure_pursuit_node pure_pursuit)  # Link with the pure_pursuit library
install(TARGETS pure_pursuit_node
        DESTINATION lib/${PROJECT_NAME})

# Executable for lifecycle_pure_pursuit
add_executable(pure_pursuit_lifecycle src/lifecycle_pure_pursuit.cpp)
ament_target_dependencies(pure_pursuit_lifecycle ${Dependencies})
target_include_directories(pure_pursuit_lifecycle PUBLIC
        ${CMAKE_SOURCE_DIR}/../../common/driverless_common/include)
target_include_directories(pure_pursuit_lifecycle PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(pure_pursuit_lifecycle pure_pursuit)  # Link with the pure_pursuit library
install(TARGETS pure_pursuit_lifecycle
        DESTINATION lib/${PROJECT_NAME})

# config folder
install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
