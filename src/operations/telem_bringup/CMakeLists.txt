cmake_minimum_required(VERSION 3.8)
project(telem_bringup)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# share folder
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
