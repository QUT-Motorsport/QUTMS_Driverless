cmake_minimum_required(VERSION 3.5)
project(qutms_hw_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(driverless_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)

# Add compile definitions for CAN_AV, CAN_VESC, and CAN_VCU
add_compile_definitions(QUTMS_CAN_AV QUTMS_CAN_VESC QUTMS_CAN_VCU)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../common/QUTMS_Embedded_Common/QUTMS_Embedded_Common/Inc
  ${CMAKE_CURRENT_SOURCE_DIR}/../../common/QUTMS_Embedded_Common/include
)

set(SOURCES
  src/SocketCAN.cpp
  src/qev_stepper_interface.cpp
  src/encos_steering_interface.cpp
  src/vcu_driving_interface.cpp
  src/sevcon_driving_interface.cpp
  src/dti_driving_interface.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../common/QUTMS_Embedded_Common/QUTMS_Embedded_Common/Src/CAN_AV.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../common/QUTMS_Embedded_Common/QUTMS_Embedded_Common/Src/CAN_VESC.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../common/QUTMS_Embedded_Common/QUTMS_Embedded_Common/Src/CAN_VCU.c
)

# Mark the C source files to compile as C++ if needed, or compile as C
set_source_files_properties(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../common/QUTMS_Embedded_Common/QUTMS_Embedded_Common/Src/CAN_AV.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../common/QUTMS_Embedded_Common/QUTMS_Embedded_Common/Src/CAN_VESC.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../common/QUTMS_Embedded_Common/QUTMS_Embedded_Common/Src/CAN_VCU.c
  PROPERTIES LANGUAGE CXX
)

add_library(${PROJECT_NAME} SHARED
  ${SOURCES}
)

target_link_libraries(${PROJECT_NAME}
  # Custom libraries or specific target linking if needed
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  driverless_msgs
  diagnostic_msgs
  pluginlib
  realtime_tools
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
endif()

# Export plugins descriptor xml
pluginlib_export_plugin_description_file(hardware_interface qutms_hw_interfaces.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  driverless_msgs
  diagnostic_msgs
  pluginlib
  realtime_tools
)

if(BUILD_TESTING)
  ament_add_gmock(test_qev_stepper_interface test/test_qev_stepper_interface.cpp)
  target_link_libraries(test_qev_stepper_interface ${PROJECT_NAME})

  ament_add_gmock(test_encos_steering_interface test/test_encos_steering_interface.cpp)
  target_link_libraries(test_encos_steering_interface ${PROJECT_NAME})

  ament_add_gmock(test_vcu_driving_interface test/test_vcu_driving_interface.cpp)
  target_link_libraries(test_vcu_driving_interface ${PROJECT_NAME})

  ament_add_gmock(test_sevcon_driving_interface test/test_sevcon_driving_interface.cpp)
  target_link_libraries(test_sevcon_driving_interface ${PROJECT_NAME})

  ament_add_gmock(test_dti_driving_interface test/test_dti_driving_interface.cpp)
  target_link_libraries(test_dti_driving_interface ${PROJECT_NAME})
endif()

ament_package()
