cmake_minimum_required(VERSION 3.5)
project(qutms_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# further dependencies 
find_package(rosidl_default_generators REQUIRED)

# Messages and Services
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/ConeScan.msg"
    "msg/ConeData.msg"
    # DEPENDENCIES std_msgs
)

ament_package()