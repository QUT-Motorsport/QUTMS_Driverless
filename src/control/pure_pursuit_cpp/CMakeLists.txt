cmake_minimum_required(VERSION 3.5)
project(pure_pursuit_cpp)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(driverless_msgs REQUIRED)
find_package(tf2 REQUIRED)

SET_SOURCE_FILES_PROPERTIES(${SOURCES} PROPERTIES LANGUAGE CXX)

add_executable(pure_pursuit_cpp src/node_pure_pursuit_cpp.cpp ${SOURCES})
target_include_directories(pure_pursuit_cpp
        PUBLIC include
        PUBLIC ${CMAKE_SOURCE_DIR}/../../common/driverless_common/include)
ament_target_dependencies(pure_pursuit_cpp
        rclcpp
        driverless_msgs
        ackermann_msgs
        tf2)

target_include_directories(pure_pursuit_cpp PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

install(TARGETS pure_pursuit_cpp
        DESTINATION lib/${PROJECT_NAME})

# config folder
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})

ament_package()
