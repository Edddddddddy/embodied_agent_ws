cmake_minimum_required(VERSION 3.16)
project(embodied_navigation)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(embodied_agent_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(dynamic_obstacle_tracker_core src/dynamic_obstacle_tracker.cpp)
target_include_directories(dynamic_obstacle_tracker_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(constant_velocity_predictor src/constant_velocity_predictor.cpp)
target_include_directories(constant_velocity_predictor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(dynamic_obstacle_tracker_node src/dynamic_obstacle_tracker_node.cpp)
target_link_libraries(dynamic_obstacle_tracker_node dynamic_obstacle_tracker_core)
ament_target_dependencies(dynamic_obstacle_tracker_node
  embodied_agent_interfaces geometry_msgs rclcpp)

add_library(predicted_obstacle_layer SHARED src/predicted_obstacle_layer.cpp)
target_link_libraries(predicted_obstacle_layer constant_velocity_predictor)
target_include_directories(predicted_obstacle_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(predicted_obstacle_layer
  embodied_agent_interfaces geometry_msgs nav2_costmap_2d pluginlib rclcpp tf2 tf2_geometry_msgs)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

install(TARGETS constant_velocity_predictor dynamic_obstacle_tracker_core
  dynamic_obstacle_tracker_node predicted_obstacle_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_lint_cmake()
  ament_xmllint()
  ament_add_gtest(test_dynamic_obstacle_tracker test/test_dynamic_obstacle_tracker.cpp)
  target_link_libraries(test_dynamic_obstacle_tracker dynamic_obstacle_tracker_core)
  ament_add_gtest(test_constant_velocity_predictor test/test_constant_velocity_predictor.cpp)
  target_link_libraries(test_constant_velocity_predictor constant_velocity_predictor)
endif()

ament_export_include_directories(include)
ament_export_libraries(constant_velocity_predictor dynamic_obstacle_tracker_core)
ament_package()
