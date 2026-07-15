cmake_minimum_required(VERSION 3.16)
project(embodied_slam)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization thread)
find_package(diagnostic_msgs REQUIRED)
find_package(embodied_agent_interfaces REQUIRED)
find_package(embodied_agent_middleware REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(GTSAM REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(std_msgs REQUIRED)
find_package(TBB REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(embodied_slam_core
  src/closed_loop_controller.cpp
  src/drift_model.cpp
  src/lidar_loop_descriptor.cpp
  src/lidar_loop_constraint_gate.cpp
  src/lidar_loop_sequence_consistency.cpp
  src/lidar_loop_temporal_consistency.cpp
  src/lidar_loop_runtime.cpp
  src/lidar_loop_verifier.cpp
  src/lidar_scan_corpus.cpp
  src/lidar_scan_matcher.cpp
  src/lidar_submap_builder.cpp
  src/loop_frontend_diagnostics.cpp
  src/loop_constraint_adapter.cpp
  src/scan_overlap_validator.cpp
  src/trajectory_metrics.cpp
)
set_target_properties(embodied_slam_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
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
target_link_libraries(embodied_slam_gtsam_solver embodied_slam_core embodied_slam_gtsam_core)
ament_target_dependencies(embodied_slam_gtsam_solver
  pluginlib rclcpp_lifecycle slam_toolbox)
pluginlib_export_plugin_description_file(slam_toolbox gtsam_solver_plugin.xml)

add_executable(gtsam_graph_optimize src/gtsam_graph_optimize.cpp)
target_link_libraries(gtsam_graph_optimize embodied_slam_gtsam_core)

add_executable(lidar_loop_candidates src/lidar_loop_candidates.cpp)
target_link_libraries(lidar_loop_candidates embodied_slam_core)

add_executable(lidar_shadow_scan_match src/lidar_shadow_scan_match.cpp)
target_link_libraries(lidar_shadow_scan_match embodied_slam_core)

add_executable(lidar_loop_temporal_replay src/lidar_loop_temporal_replay.cpp)
target_link_libraries(lidar_loop_temporal_replay embodied_slam_core)

add_library(lidar_loop_candidate_component SHARED src/lidar_loop_candidate_node.cpp)
target_link_libraries(lidar_loop_candidate_component embodied_slam_core)
ament_target_dependencies(lidar_loop_candidate_component
  embodied_agent_interfaces embodied_agent_middleware rclcpp rclcpp_components
  rclcpp_lifecycle sensor_msgs)
rclcpp_components_register_nodes(
  lidar_loop_candidate_component "embodied_slam::LidarLoopCandidateNode")

add_executable(lidar_loop_candidate_node src/lidar_loop_candidate_main.cpp)
target_link_libraries(lidar_loop_candidate_node lidar_loop_candidate_component)
ament_target_dependencies(lidar_loop_candidate_node rclcpp rclcpp_lifecycle)

add_library(lidar_loop_verifier_component SHARED src/lidar_loop_verifier_node.cpp)
target_link_libraries(lidar_loop_verifier_component embodied_slam_core)
ament_target_dependencies(lidar_loop_verifier_component
  embodied_agent_interfaces embodied_agent_middleware geometry_msgs nav_msgs rclcpp
  rclcpp_components rclcpp_lifecycle sensor_msgs)
rclcpp_components_register_nodes(
  lidar_loop_verifier_component "embodied_slam::LidarLoopVerifierNode")

add_executable(lidar_loop_verifier_node src/lidar_loop_verifier_main.cpp)
target_link_libraries(lidar_loop_verifier_node lidar_loop_verifier_component)
ament_target_dependencies(lidar_loop_verifier_node rclcpp rclcpp_lifecycle)

add_library(lidar_loop_constraint_gate_component SHARED
  src/lidar_loop_constraint_gate_node.cpp)
target_link_libraries(lidar_loop_constraint_gate_component embodied_slam_core)
ament_target_dependencies(lidar_loop_constraint_gate_component
  embodied_agent_interfaces embodied_agent_middleware rclcpp rclcpp_components
  rclcpp_lifecycle)
rclcpp_components_register_nodes(
  lidar_loop_constraint_gate_component "embodied_slam::LidarLoopConstraintGateNode")

add_executable(lidar_loop_constraint_gate_node
  src/lidar_loop_constraint_gate_main.cpp)
target_link_libraries(lidar_loop_constraint_gate_node
  lidar_loop_constraint_gate_component)
ament_target_dependencies(lidar_loop_constraint_gate_node rclcpp rclcpp_lifecycle)

add_executable(odom_drift_injector_node src/odom_drift_injector_node.cpp)
target_link_libraries(odom_drift_injector_node embodied_slam_core)
ament_target_dependencies(odom_drift_injector_node
  diagnostic_msgs embodied_agent_middleware geometry_msgs nav_msgs rclcpp sensor_msgs tf2 tf2_ros)

add_executable(closed_loop_driver_node src/closed_loop_driver_node.cpp)
target_link_libraries(closed_loop_driver_node embodied_slam_core)
ament_target_dependencies(closed_loop_driver_node
  embodied_agent_middleware geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs tf2)

add_executable(instrumented_async_slam_toolbox_node
  src/instrumented_async_slam_toolbox_node.cpp)
# slam_toolbox 的 Jazzy 二进制包没有为这些实现库导出 CMake target；只定位本节点
# 直接使用的三个库，避免把整个 legacy *_LIBRARIES 依赖闭包重复塞进链接命令。
find_library(SLAM_TOOLBOX_ASYNC_LIBRARY NAMES async_slam_toolbox
  HINTS "${slam_toolbox_DIR}/../../../lib" REQUIRED)
find_library(SLAM_TOOLBOX_COMMON_LIBRARY NAMES toolbox_common
  HINTS "${slam_toolbox_DIR}/../../../lib" REQUIRED)
find_library(SLAM_TOOLBOX_KARTO_LIBRARY NAMES kartoSlamToolbox
  HINTS "${slam_toolbox_DIR}/../../../lib" REQUIRED)
target_link_libraries(instrumented_async_slam_toolbox_node
  embodied_slam_core
  ${SLAM_TOOLBOX_ASYNC_LIBRARY}
  ${SLAM_TOOLBOX_COMMON_LIBRARY}
  ${SLAM_TOOLBOX_KARTO_LIBRARY}
  Boost::serialization Boost::thread TBB::tbb)
ament_target_dependencies(instrumented_async_slam_toolbox_node
  embodied_agent_interfaces embodied_agent_middleware rclcpp rclcpp_lifecycle
  sensor_msgs slam_toolbox)

install(TARGETS
  embodied_slam_core
  embodied_slam_gtsam_core
  embodied_slam_gtsam_solver
  gtsam_graph_optimize
  lidar_loop_candidates
  lidar_shadow_scan_match
  lidar_loop_temporal_replay
  lidar_loop_candidate_component
  lidar_loop_candidate_node
  lidar_loop_verifier_component
  lidar_loop_verifier_node
  lidar_loop_constraint_gate_component
  lidar_loop_constraint_gate_node
  odom_drift_injector_node
  closed_loop_driver_node
  instrumented_async_slam_toolbox_node
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
  ament_add_gtest(test_loop_frontend_diagnostics test/test_loop_frontend_diagnostics.cpp)
  target_link_libraries(test_loop_frontend_diagnostics embodied_slam_core)
  ament_add_gtest(test_lidar_loop_descriptor test/test_lidar_loop_descriptor.cpp)
  target_link_libraries(test_lidar_loop_descriptor embodied_slam_core)
  ament_add_gtest(test_lidar_loop_runtime test/test_lidar_loop_runtime.cpp)
  target_link_libraries(test_lidar_loop_runtime embodied_slam_core)
  ament_add_gtest(test_lidar_loop_verifier test/test_lidar_loop_verifier.cpp)
  target_link_libraries(test_lidar_loop_verifier embodied_slam_core)
  ament_add_gtest(test_lidar_loop_constraint_gate
    test/test_lidar_loop_constraint_gate.cpp)
  target_link_libraries(test_lidar_loop_constraint_gate embodied_slam_core)
  ament_add_gtest(test_lidar_loop_sequence_consistency
    test/test_lidar_loop_sequence_consistency.cpp)
  target_link_libraries(test_lidar_loop_sequence_consistency embodied_slam_core)
  ament_add_gtest(test_lidar_loop_temporal_consistency
    test/test_lidar_loop_temporal_consistency.cpp)
  target_link_libraries(test_lidar_loop_temporal_consistency embodied_slam_core)
  ament_add_gtest(test_loop_constraint_adapter
    test/test_loop_constraint_adapter.cpp)
  target_link_libraries(test_loop_constraint_adapter embodied_slam_core)
  ament_add_gtest(test_lidar_scan_matcher test/test_lidar_scan_matcher.cpp)
  target_link_libraries(test_lidar_scan_matcher embodied_slam_core)
  ament_add_gtest(test_lidar_submap_builder test/test_lidar_submap_builder.cpp)
  target_link_libraries(test_lidar_submap_builder embodied_slam_core)
  ament_add_gtest(test_scan_overlap_validator test/test_scan_overlap_validator.cpp)
  target_link_libraries(test_scan_overlap_validator embodied_slam_core)
  add_test(
    NAME test_gtsam_graph_optimize_cli
    COMMAND gtsam_graph_optimize
      ${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/synthetic_pose_graph.txt
      ${CMAKE_CURRENT_BINARY_DIR}/synthetic_pose_graph.tum
      cauchy 1.0 true 2 20
  )
  add_test(
    NAME test_gtsam_graph_optimize_overlap_cli
    COMMAND gtsam_graph_optimize
      ${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/synthetic_pose_graph_overlap.txt
      ${CMAKE_CURRENT_BINARY_DIR}/synthetic_pose_graph_overlap.tum
      cauchy 1.0 true 2 20 false 2.0 0.785398 true 0.65 0.0
  )
  set_tests_properties(
    test_gtsam_graph_optimize_overlap_cli
    PROPERTIES PASS_REGULAR_EXPRESSION "\"scan_overlap_rejected_constraints\":1"
  )
  add_test(
    NAME test_gtsam_graph_optimize_switchable_cli
    COMMAND gtsam_graph_optimize
      ${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/synthetic_pose_graph_switchable.txt
      ${CMAKE_CURRENT_BINARY_DIR}/synthetic_pose_graph_switchable.tum
      none 0.0 true 5 50 false 2.0 0.785398 false 0.65 0.0 true 1.0 0.5
  )
  set_tests_properties(
    test_gtsam_graph_optimize_switchable_cli
    PROPERTIES PASS_REGULAR_EXPRESSION "\"switch_suppressed_constraints\":1"
  )
  add_test(
    NAME test_lidar_loop_candidates_cli
    COMMAND lidar_loop_candidates
      ${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/lidar_scan_corpus.txt
      ${CMAKE_CURRENT_BINARY_DIR}/lidar_loop_candidates.jsonl
      4 8 10.0 4 60.0 10 2 0.0
  )
  set_tests_properties(
    test_lidar_loop_candidates_cli
    PROPERTIES PASS_REGULAR_EXPRESSION "\"emitted_candidates\":3"
  )
  add_test(
    NAME test_lidar_shadow_scan_match_cli
    COMMAND lidar_shadow_scan_match
      ${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/lidar_scan_corpus.txt
      ${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/lidar_shadow_pairs.txt
      ${CMAKE_CURRENT_BINARY_DIR}/lidar_shadow_matches.jsonl
      1 4
  )
  set_tests_properties(
    test_lidar_shadow_scan_match_cli
    PROPERTIES PASS_REGULAR_EXPRESSION "\"accepted\":1"
  )
  add_test(
    NAME test_lidar_shadow_submap_match_cli
    COMMAND lidar_shadow_scan_match
      ${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/lidar_scan_corpus.txt
      ${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/lidar_shadow_pairs.txt
      ${CMAKE_CURRENT_BINARY_DIR}/lidar_shadow_submap_matches.jsonl
      --point-stride 1
      --minimum-points 4
      --submap-odometry
        ${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/lidar_odometry_corpus.txt
      --submap-half-window 1
      --submap-max-time-delta 100.0
      --submap-point-stride 1
      --submap-minimum-scans 2
      --submap-minimum-points 4
  )
  set_tests_properties(
    test_lidar_shadow_submap_match_cli
    PROPERTIES PASS_REGULAR_EXPRESSION "\"matching_mode\":\"scan_to_submap\""
  )
  add_test(
    NAME test_lidar_loop_temporal_replay_cli
    COMMAND lidar_loop_temporal_replay
      ${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/lidar_loop_temporal_observations.txt
      ${CMAKE_CURRENT_BINARY_DIR}/lidar_loop_temporal_decisions.jsonl
  )
  set_tests_properties(
    test_lidar_loop_temporal_replay_cli
    PROPERTIES PASS_REGULAR_EXPRESSION "\"approved\":1"
  )
  add_test(
    NAME test_lidar_loop_sequence_replay_cli
    COMMAND lidar_loop_temporal_replay
      ${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/lidar_loop_sequence_observations.txt
      ${CMAKE_CURRENT_BINARY_DIR}/lidar_loop_sequence_decisions.jsonl
      --multi-hypothesis
      --minimum-confirmations 3
      --maximum-pair-age-delta 0.25
      --maximum-translation-delta 0.35
      --maximum-yaw-delta 0.20
  )
  set_tests_properties(
    test_lidar_loop_sequence_replay_cli
    PROPERTIES PASS_REGULAR_EXPRESSION "\"mode\":\"multi_hypothesis\".*\"approved\":1"
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(embodied_slam_core embodied_slam_gtsam_core)
ament_package()
