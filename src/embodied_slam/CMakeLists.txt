cmake_minimum_required(VERSION 3.16)
project(embodied_slam)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(embodied_agent_middleware REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(GTSAM REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(embodied_slam_core
  src/closed_loop_controller.cpp
  src/drift_model.cpp
  src/trajectory_metrics.cpp
)
target_include_directories(embodied_slam_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(embodied_slam_gtsam_core src/gtsam_pose_graph.cpp)
set_target_properties(embodied_slam_gtsam_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(embodied_slam_gtsam_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(embodied_slam_gtsam_core PUBLIC gtsam)

add_library(embodied_slam_gtsam_solver SHARED src/gtsam_scan_solver.cpp)
target_include_directories(embodied_slam_gtsam_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(embodied_slam_gtsam_solver embodied_slam_gtsam_core)
ament_target_dependencies(embodied_slam_gtsam_solver
  pluginlib rclcpp_lifecycle slam_toolbox)
pluginlib_export_plugin_description_file(slam_toolbox gtsam_solver_plugin.xml)

add_executable(odom_drift_injector_node src/odom_drift_injector_node.cpp)
target_link_libraries(odom_drift_injector_node embodied_slam_core)
ament_target_dependencies(odom_drift_injector_node
  diagnostic_msgs embodied_agent_middleware geometry_msgs nav_msgs rclcpp sensor_msgs tf2 tf2_ros)

add_executable(closed_loop_driver_node src/closed_loop_driver_node.cpp)
target_link_libraries(closed_loop_driver_node embodied_slam_core)
ament_target_dependencies(closed_loop_driver_node
  embodied_agent_middleware geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs tf2)

install(TARGETS
  embodied_slam_core
  embodied_slam_gtsam_core
  embodied_slam_gtsam_solver
  odom_drift_injector_node
  closed_loop_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_lint_cmake()
  ament_xmllint()

  ament_add_gtest(test_drift_model test/test_drift_model.cpp)
  target_link_libraries(test_drift_model embodied_slam_core)
  ament_add_gtest(test_trajectory_metrics test/test_trajectory_metrics.cpp)
  target_link_libraries(test_trajectory_metrics embodied_slam_core)
  ament_add_gtest(test_closed_loop_controller test/test_closed_loop_controller.cpp)
  target_link_libraries(test_closed_loop_controller embodied_slam_core)
  ament_add_gtest(test_gtsam_pose_graph test/test_gtsam_pose_graph.cpp)
  target_link_libraries(test_gtsam_pose_graph embodied_slam_gtsam_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(embodied_slam_core embodied_slam_gtsam_core)
ament_package()
