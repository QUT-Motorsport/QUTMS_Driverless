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

SET_SOURCE_FILES_PROPERTIES(${SOURCES} PROPERTIES LANGUAGE CXX)

set(Nodes pure_pursuit_cpp_node pure_pursuit_cpp_lifecycle)
set(Srcs src/node_pure_pursuit_cpp.cpp src/lifecycle_pure_pursuit_cpp.cpp)

foreach(Node Src IN ZIP_LISTS Nodes Srcs)
    add_executable(${Node} ${Src})
    ament_target_dependencies(${Node} ${Dependencies})
    target_include_directories(${Node} PUBLIC
        ${CMAKE_SOURCE_DIR}/../../common/driverless_common/include)
    target_include_directories(${Node} PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)
    install(TARGETS ${Node}
            DESTINATION lib/${PROJECT_NAME})
endforeach()

# config folder
install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
