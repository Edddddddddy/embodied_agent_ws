cmake_minimum_required(VERSION 3.16)
project(embodied_agent_middleware)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(embodied_agent_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_library(embodied_agent_middleware INTERFACE)
target_include_directories(embodied_agent_middleware INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(embodied_agent_middleware INTERFACE rclcpp::rclcpp)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS embodied_agent_middleware EXPORT embodied_agent_middlewareTargets)

add_library(component_health_core src/component_health_registry.cpp)
target_include_directories(component_health_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(system_readiness_node src/system_readiness_node.cpp)
target_link_libraries(
  system_readiness_node component_health_core embodied_agent_middleware)
ament_target_dependencies(
  system_readiness_node embodied_agent_interfaces rclcpp)

install(TARGETS
  component_health_core
  system_readiness_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_lint_cmake()
  ament_xmllint()
  ament_add_gtest(test_qos_profiles test/test_qos_profiles.cpp)
  target_link_libraries(test_qos_profiles embodied_agent_middleware)
  ament_add_gtest(
    test_component_health_registry test/test_component_health_registry.cpp)
  target_link_libraries(test_component_health_registry component_health_core)
endif()

ament_export_targets(embodied_agent_middlewareTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(component_health_core)
ament_export_dependencies(embodied_agent_interfaces rclcpp)
ament_package()
