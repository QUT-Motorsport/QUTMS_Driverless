cmake_minimum_required(VERSION 3.8)
project(mpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)


add_executable(parra_test src/parra_test.cpp)
add_executable(coord_transform src/coord_transform.cpp)
add_executable(track_transform src/track_transform.cpp)
target_include_directories(parra_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(parra_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(coord_transform PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(track_transform PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(parra_test PRIVATE Threads::Threads)
target_link_libraries(coord_transform PRIVATE Threads::Threads)
target_link_libraries(track_transform PRIVATE Threads::Threads)

install(TARGETS parra_test coord_transform track_transform
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
