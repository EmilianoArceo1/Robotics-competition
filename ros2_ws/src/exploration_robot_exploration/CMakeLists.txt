cmake_minimum_required(VERSION 3.16)
project(exploration_robot_exploration LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(exploration_robot_control REQUIRED)

add_library(exploration_core
  src/coordination/control.cpp
  src/mapping/belief_map.cpp
  src/objective_assignment/frontiers/frontiers.cpp
  src/objective_assignment/frontiers/nearest_frontier.cpp
  src/planning/track.cpp
  src/planning/waypoints.cpp
  src/sensing/sensor.cpp)
target_compile_features(exploration_core PUBLIC cxx_std_17)
target_include_directories(exploration_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(exploration_core exploration_robot_control)

install(TARGETS exploration_core
  EXPORT export_exploration_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_exploration_core test/test_exploration_core.cpp)
  target_link_libraries(test_exploration_core exploration_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(exploration_core)
ament_export_dependencies(exploration_robot_control)
ament_export_targets(export_exploration_core HAS_LIBRARY_TARGET)
ament_package()
