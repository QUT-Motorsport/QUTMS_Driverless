cmake_minimum_required(VERSION 3.18)
project(driverless_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# messages
set(msg_files
  "msg/Can.msg"
  "msg/Cone.msg"
  "msg/ConeDetectionStamped.msg"
  "msg/ConeWithCovariance.msg"
  "msg/DebugMsg.msg"
  "msg/DoubleMatrix.msg"
  "msg/DrivingDynamics1.msg"
  "msg/Float32Stamped.msg"
  "msg/Int32Stamped.msg"
  "msg/PathPoint.msg"
  "msg/PathStamped.msg"
  "msg/RES.msg"
  "msg/Shutdown.msg"
  "msg/State.msg"
  "msg/SystemStatus.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
