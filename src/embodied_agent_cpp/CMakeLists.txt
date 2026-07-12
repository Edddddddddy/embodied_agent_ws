cmake_minimum_required(VERSION 3.16)
project(embodied_agent_cpp)

add_compile_options(-Wall -Wextra -Wpedantic)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(embodied_agent_interfaces REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)

add_library(embodied_agent_core
  src/action_scheduler.cpp
  src/action_validator.cpp
  src/audio_processing.cpp
  src/hardware_protocol.cpp
  src/hardware_transport.cpp
  src/typed_action_client_contract.cpp
)
target_include_directories(embodied_agent_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(embodied_agent_core PUBLIC nlohmann_json::nlohmann_json)
ament_target_dependencies(embodied_agent_core PUBLIC embodied_agent_interfaces)

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
ament_target_dependencies(
  action_guard embodied_agent_interfaces lifecycle_msgs rclcpp rclcpp_lifecycle std_msgs)

add_executable(typed_action_bridge src/typed_action_bridge_node.cpp)
target_link_libraries(typed_action_bridge embodied_agent_core)
ament_target_dependencies(
  typed_action_bridge diagnostic_msgs embodied_agent_interfaces rclcpp rclcpp_action)

add_executable(typed_action_demo_client src/typed_action_demo_client.cpp)
target_link_libraries(
  typed_action_demo_client embodied_agent_core nlohmann_json::nlohmann_json)
ament_target_dependencies(
  typed_action_demo_client embodied_agent_interfaces rclcpp rclcpp_action)

add_executable(robot_action_stub src/robot_action_stub_node.cpp)
target_link_libraries(robot_action_stub nlohmann_json::nlohmann_json)
ament_target_dependencies(robot_action_stub rclcpp std_msgs embodied_agent_interfaces)

add_executable(hardware_controller src/hardware_controller_node.cpp)
target_link_libraries(hardware_controller embodied_agent_core nlohmann_json::nlohmann_json)
ament_target_dependencies(hardware_controller embodied_agent_interfaces rclcpp std_msgs)

set(SUMMER_TTS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/SummerTTS" CACHE PATH "SummerTTS source root")
if(EXISTS "${SUMMER_TTS_ROOT}/include/SynthesizerTrn.h")
  find_package(OpenMP)
  set(SUMMER_TTS_SOURCES
    "${SUMMER_TTS_ROOT}/src/tn/glog/src/demangle.cc"
    "${SUMMER_TTS_ROOT}/src/tn/glog/src/logging.cc"
    "${SUMMER_TTS_ROOT}/src/tn/glog/src/raw_logging.cc"
    "${SUMMER_TTS_ROOT}/src/tn/glog/src/symbolize.cc"
    "${SUMMER_TTS_ROOT}/src/tn/glog/src/utilities.cc"
    "${SUMMER_TTS_ROOT}/src/tn/glog/src/vlog_is_on.cc"
    "${SUMMER_TTS_ROOT}/src/tn/glog/src/signalhandler.cc"
    "${SUMMER_TTS_ROOT}/src/tn/gflags/src/gflags.cc"
    "${SUMMER_TTS_ROOT}/src/tn/gflags/src/gflags_reporting.cc"
    "${SUMMER_TTS_ROOT}/src/tn/gflags/src/gflags_completions.cc"
    "${SUMMER_TTS_ROOT}/src/tn/openfst/src/lib/compat.cc"
    "${SUMMER_TTS_ROOT}/src/tn/openfst/src/lib/flags.cc"
    "${SUMMER_TTS_ROOT}/src/tn/openfst/src/lib/fst.cc"
    "${SUMMER_TTS_ROOT}/src/tn/openfst/src/lib/fst-types.cc"
    "${SUMMER_TTS_ROOT}/src/tn/openfst/src/lib/mapped-file.cc"
    "${SUMMER_TTS_ROOT}/src/tn/openfst/src/lib/properties.cc"
    "${SUMMER_TTS_ROOT}/src/tn/openfst/src/lib/symbol-table.cc"
    "${SUMMER_TTS_ROOT}/src/tn/openfst/src/lib/symbol-table-ops.cc"
    "${SUMMER_TTS_ROOT}/src/tn/openfst/src/lib/util.cc"
    "${SUMMER_TTS_ROOT}/src/tn/openfst/src/lib/weight.cc"
    "${SUMMER_TTS_ROOT}/src/tn/processor.cc"
    "${SUMMER_TTS_ROOT}/src/tn/token_parser.cc"
    "${SUMMER_TTS_ROOT}/src/tn/utf8_string.cc"
    "${SUMMER_TTS_ROOT}/src/engipa/EnglishText2Id.cpp"
    "${SUMMER_TTS_ROOT}/src/engipa/InitIPASymbols.cpp"
    "${SUMMER_TTS_ROOT}/src/engipa/alphabet.cpp"
    "${SUMMER_TTS_ROOT}/src/engipa/ipa.cpp"
    "${SUMMER_TTS_ROOT}/src/hz2py/hanzi2phoneid.cpp"
    "${SUMMER_TTS_ROOT}/src/hz2py/Hanz2Piny.cpp"
    "${SUMMER_TTS_ROOT}/src/hz2py/pinyinmap.cpp"
    "${SUMMER_TTS_ROOT}/src/nn_op/nn_conv1d.cpp"
    "${SUMMER_TTS_ROOT}/src/nn_op/nn_softmax.cpp"
    "${SUMMER_TTS_ROOT}/src/nn_op/nn_layer_norm.cpp"
    "${SUMMER_TTS_ROOT}/src/nn_op/nn_relu.cpp"
    "${SUMMER_TTS_ROOT}/src/nn_op/nn_gelu.cpp"
    "${SUMMER_TTS_ROOT}/src/nn_op/nn_tanh.cpp"
    "${SUMMER_TTS_ROOT}/src/nn_op/nn_flip.cpp"
    "${SUMMER_TTS_ROOT}/src/nn_op/nn_cumsum.cpp"
    "${SUMMER_TTS_ROOT}/src/nn_op/nn_softplus.cpp"
    "${SUMMER_TTS_ROOT}/src/nn_op/nn_clamp_min.cpp"
    "${SUMMER_TTS_ROOT}/src/nn_op/nn_sigmoid.cpp"
    "${SUMMER_TTS_ROOT}/src/nn_op/nn_conv1d_transposed.cpp"
    "${SUMMER_TTS_ROOT}/src/nn_op/nn_leaky_relu.cpp"
    "${SUMMER_TTS_ROOT}/src/platform/tts_file_io.cpp"
    "${SUMMER_TTS_ROOT}/src/platform/tts_logger.cpp"
    "${SUMMER_TTS_ROOT}/src/utils/utils.cpp"
    "${SUMMER_TTS_ROOT}/src/modules/iStft.cpp"
    "${SUMMER_TTS_ROOT}/src/modules/hann.cpp"
    "${SUMMER_TTS_ROOT}/src/modules/attention_encoder.cpp"
    "${SUMMER_TTS_ROOT}/src/modules/multi_head_attention.cpp"
    "${SUMMER_TTS_ROOT}/src/modules/ffn.cpp"
    "${SUMMER_TTS_ROOT}/src/modules/ConvFlow.cpp"
    "${SUMMER_TTS_ROOT}/src/modules/DDSConv.cpp"
    "${SUMMER_TTS_ROOT}/src/modules/ElementwiseAffine.cpp"
    "${SUMMER_TTS_ROOT}/src/modules/random_gen.cpp"
    "${SUMMER_TTS_ROOT}/src/modules/ResidualCouplingLayer.cpp"
    "${SUMMER_TTS_ROOT}/src/modules/ResBlock1.cpp"
    "${SUMMER_TTS_ROOT}/src/modules/WN.cpp"
    "${SUMMER_TTS_ROOT}/src/modules/pqmf.cpp"
    "${SUMMER_TTS_ROOT}/src/models/TextEncoder.cpp"
    "${SUMMER_TTS_ROOT}/src/models/StochasticDurationPredictor.cpp"
    "${SUMMER_TTS_ROOT}/src/models/FixDurationPredictor.cpp"
    "${SUMMER_TTS_ROOT}/src/models/DurationPredictor_base.cpp"
    "${SUMMER_TTS_ROOT}/src/models/ResidualCouplingBlock.cpp"
    "${SUMMER_TTS_ROOT}/src/models/Generator_base.cpp"
    "${SUMMER_TTS_ROOT}/src/models/Generator_hifigan.cpp"
    "${SUMMER_TTS_ROOT}/src/models/Generator_MS.cpp"
    "${SUMMER_TTS_ROOT}/src/models/Generator_Istft.cpp"
    "${SUMMER_TTS_ROOT}/src/models/Generator_MBB.cpp"
    "${SUMMER_TTS_ROOT}/src/models/SynthesizerTrn.cpp"
  )
  add_library(summer_tts_component SHARED
    src/summer_tts_service_node.cpp
    ${SUMMER_TTS_SOURCES}
  )
  target_include_directories(summer_tts_component PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    "${SUMMER_TTS_ROOT}/eigen-3.4.0"
    "${SUMMER_TTS_ROOT}/include"
    "${SUMMER_TTS_ROOT}/src/header"
    "${SUMMER_TTS_ROOT}/src/tn/header"
  )
  # SummerTTS 上游部分头文件在 GCC 13 下缺少 cstdint；强制 include 避免依赖本地补丁状态。
  target_compile_options(summer_tts_component PRIVATE -include cstdint)
  if(OpenMP_CXX_FOUND)
    target_link_libraries(summer_tts_component OpenMP::OpenMP_CXX)
  endif()
  ament_target_dependencies(
    summer_tts_component embodied_agent_interfaces rclcpp rclcpp_components)
  rclcpp_components_register_nodes(
    summer_tts_component "embodied_agent_cpp::SummerTtsServiceNode")

  add_executable(summer_tts_service src/summer_tts_service_main.cpp)
  target_include_directories(summer_tts_service PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(summer_tts_service summer_tts_component)
  ament_target_dependencies(summer_tts_service rclcpp)
else()
  message(WARNING "SummerTTS source not found at ${SUMMER_TTS_ROOT}; summer_tts_service will not be built")
endif()

install(TARGETS
  embodied_agent_core
  audio_frontend
  action_guard
  typed_action_bridge
  typed_action_demo_client
  robot_action_stub
  hardware_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
if(TARGET summer_tts_component)
  install(TARGETS
    summer_tts_component
    summer_tts_service
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

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
  ament_add_gtest(test_audio_processing test/test_audio_processing.cpp)
  target_link_libraries(test_audio_processing embodied_agent_core)
  ament_add_gtest(test_action_validator test/test_action_validator.cpp)
  target_link_libraries(test_action_validator embodied_agent_core)
  ament_add_gtest(test_action_scheduler test/test_action_scheduler.cpp)
  target_link_libraries(test_action_scheduler embodied_agent_core)
  ament_add_gtest(test_hardware_protocol test/test_hardware_protocol.cpp)
  target_link_libraries(test_hardware_protocol embodied_agent_core)
  ament_add_gtest(test_hardware_transport test/test_hardware_transport.cpp)
  target_link_libraries(test_hardware_transport embodied_agent_core)
  ament_add_gtest(
    test_typed_action_client_contract test/test_typed_action_client_contract.cpp)
  target_link_libraries(test_typed_action_client_contract embodied_agent_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(embodied_agent_core)
ament_export_dependencies(embodied_agent_interfaces)
ament_package()
