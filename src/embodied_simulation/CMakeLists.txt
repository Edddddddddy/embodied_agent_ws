cmake_minimum_required(VERSION 3.16)
project(embodied_simulation)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(simulation_control_core src/simulation_controller.cpp)
target_include_directories(simulation_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(simulation_control_node src/simulation_control_node.cpp)
target_link_libraries(simulation_control_node
  simulation_control_core
  nlohmann_json::nlohmann_json
)
ament_target_dependencies(simulation_control_node
  geometry_msgs rclcpp sensor_msgs std_msgs
)

install(TARGETS simulation_control_core simulation_control_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_simulation_controller test/test_simulation_controller.cpp)
  target_link_libraries(test_simulation_controller simulation_control_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(simulation_control_core)
ament_package()
