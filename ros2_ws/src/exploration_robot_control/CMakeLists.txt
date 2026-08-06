cmake_minimum_required(VERSION 3.16)
project(exploration_robot_control LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(acceleration_controller src/acceleration_controller.cpp)
target_compile_features(acceleration_controller PUBLIC cxx_std_17)
target_include_directories(acceleration_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(acceleration_controller_node src/acceleration_controller_node.cpp)
target_compile_features(acceleration_controller_node PUBLIC cxx_std_17)
target_link_libraries(acceleration_controller_node acceleration_controller)
ament_target_dependencies(acceleration_controller_node geometry_msgs rclcpp)

install(TARGETS acceleration_controller acceleration_controller_node
  EXPORT export_acceleration_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_acceleration_controller test/test_acceleration_controller.cpp)
  target_link_libraries(test_acceleration_controller acceleration_controller)
endif()

ament_export_include_directories(include)
ament_export_libraries(acceleration_controller)
ament_export_targets(export_acceleration_controller HAS_LIBRARY_TARGET)
ament_package()
