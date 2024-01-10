cmake_minimum_required(VERSION 3.5)
project(map_creation)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif ()

set(
    Deps
    ament_cmake
    rclcpp
    driverless_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    nav_msgs
)

# find dependencies
foreach(Dep ${Deps})
    find_package(${Dep} REQUIRED)
endforeach()

SET_SOURCE_FILES_PROPERTIES(${SOURCES} PROPERTIES LANGUAGE CXX)

set(
    Nodes
    cone_placement
    # boundary_interpolation
)

foreach(Node ${Nodes})
    add_executable(${Node}_node src/node_${Node}.cpp ${SOURCES})
    target_include_directories(${Node}_node
        PUBLIC include
        PUBLIC ${CMAKE_SOURCE_DIR}/../../common/driverless_common/include)
    ament_target_dependencies(${Node}_node ${Deps})

    target_include_directories(${Node}_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

    install(TARGETS ${Node}_node
        DESTINATION lib/${PROJECT_NAME})
endforeach()

# config folder
install(DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME})

ament_package()
