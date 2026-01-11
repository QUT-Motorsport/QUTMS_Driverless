cmake_minimum_required(VERSION 3.8)
project(points_maker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# PCL local build problem
find_package(Boost COMPONENTS date_time)

ament_auto_add_library(${PROJECT_NAME}_node_component SHARED src/points_maker.cpp)

target_compile_definitions(${PROJECT_NAME}_node_component PRIVATE "GROUND_PLANE_SEGMENTER_BUILDING_LIBRARY")

rclcpp_components_register_node(${PROJECT_NAME}_node_component
  PLUGIN "points_maker::PointsMakerNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

# # config folder
# install(DIRECTORY config
#     DESTINATION share/${PROJECT_NAME})

ament_auto_package()
