cmake_minimum_required(VERSION 3.16)
project(embodied_simulation)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(embodied_agent_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(simulation_control_core
  src/action_execution.cpp
  src/command_behavior_tree.cpp
  src/executor_diagnostics.cpp
  src/nav2_places.cpp
  src/node_configuration.cpp
  src/simulation_controller.cpp
)
set_target_properties(simulation_control_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON)
target_include_directories(simulation_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(simulation_control_core PUBLIC
  behaviortree_cpp diagnostic_msgs embodied_agent_interfaces geometry_msgs)

add_library(embodied_simulation_executors SHARED
  src/robot_executor_plugins.cpp
)
target_link_libraries(embodied_simulation_executors simulation_control_core)
ament_target_dependencies(embodied_simulation_executors
  ament_index_cpp embodied_agent_interfaces geometry_msgs nav2_msgs pluginlib rclcpp rclcpp_action)
pluginlib_export_plugin_description_file(
  embodied_simulation robot_executor_plugins.xml)

add_library(simulation_control_component SHARED
  src/simulation_control_node.cpp
)
target_link_libraries(simulation_control_component
  simulation_control_core
  nlohmann_json::nlohmann_json
)
ament_target_dependencies(simulation_control_component
  ament_index_cpp diagnostic_msgs embodied_agent_interfaces geometry_msgs lifecycle_msgs rclcpp rclcpp_action
  pluginlib rclcpp_components rclcpp_lifecycle sensor_msgs std_msgs
)
rclcpp_components_register_nodes(
  simulation_control_component "embodied_simulation::SimulationControlNode")

add_executable(simulation_control_node src/simulation_control_main.cpp)
target_link_libraries(simulation_control_node simulation_control_component)
ament_target_dependencies(simulation_control_node rclcpp rclcpp_lifecycle)

install(TARGETS
  simulation_control_core
  embodied_simulation_executors
  simulation_control_component
  simulation_control_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch maps rviz worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_cppcheck(EXCLUDE "test/*.cpp")
  set_tests_properties(cppcheck PROPERTIES
    ENVIRONMENT "AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1")
  ament_lint_cmake()
  ament_xmllint()
  ament_add_gtest(test_simulation_controller test/test_simulation_controller.cpp)
  target_link_libraries(test_simulation_controller simulation_control_core)
  ament_add_gtest(test_action_execution test/test_action_execution.cpp)
  target_link_libraries(test_action_execution simulation_control_core)
  ament_add_gtest(test_command_behavior_tree test/test_command_behavior_tree.cpp)
  target_link_libraries(test_command_behavior_tree simulation_control_core)
  ament_add_gtest(test_robot_executor_plugins test/test_robot_executor_plugins.cpp)
  target_link_libraries(test_robot_executor_plugins simulation_control_core)
  ament_target_dependencies(test_robot_executor_plugins pluginlib)
  ament_add_gtest(test_nav2_places test/test_nav2_places.cpp)
  target_link_libraries(test_nav2_places simulation_control_core)
  ament_add_gtest(test_node_configuration test/test_node_configuration.cpp)
  target_link_libraries(test_node_configuration simulation_control_core)
  ament_add_gtest(test_executor_diagnostics test/test_executor_diagnostics.cpp)
  target_link_libraries(test_executor_diagnostics simulation_control_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(simulation_control_core)
ament_export_dependencies(
  behaviortree_cpp diagnostic_msgs embodied_agent_interfaces geometry_msgs pluginlib)
ament_package()
