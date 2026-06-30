cmake_minimum_required(VERSION 3.16)
project(embodied_agent_cpp)

add_compile_options(-Wall -Wextra -Wpedantic)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)

add_library(embodied_agent_core
  src/action_validator.cpp
  src/audio_processing.cpp
)
target_include_directories(embodied_agent_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(embodied_agent_core PUBLIC nlohmann_json::nlohmann_json)

add_executable(audio_frontend src/audio_frontend_node.cpp)
target_include_directories(audio_frontend PRIVATE ${PORTAUDIO_INCLUDE_DIRS})
target_link_directories(audio_frontend PRIVATE ${PORTAUDIO_LIBRARY_DIRS})
target_link_libraries(audio_frontend
  embodied_agent_core
  ${PORTAUDIO_LIBRARIES}
  Threads::Threads
)
ament_target_dependencies(audio_frontend rclcpp std_msgs)

add_executable(action_guard src/action_guard_node.cpp)
target_link_libraries(action_guard embodied_agent_core)
ament_target_dependencies(action_guard rclcpp std_msgs)

add_executable(robot_action_stub src/robot_action_stub_node.cpp)
target_link_libraries(robot_action_stub nlohmann_json::nlohmann_json)
ament_target_dependencies(robot_action_stub rclcpp std_msgs)

install(TARGETS
  embodied_agent_core
  audio_frontend
  action_guard
  robot_action_stub
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_audio_processing test/test_audio_processing.cpp)
  target_link_libraries(test_audio_processing embodied_agent_core)
  ament_add_gtest(test_action_validator test/test_action_validator.cpp)
  target_link_libraries(test_action_validator embodied_agent_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(embodied_agent_core)
ament_package()
