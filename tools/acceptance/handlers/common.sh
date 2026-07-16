#!/usr/bin/env bash
# Internal command implementations. Public routing lives in the Python registry.
# shellcheck shell=bash

require_file() {
  if [[ ! -s "$1" ]]; then
    echo "MISSING: $1" >&2
    return 1
  fi
  echo "READY: $1"
}

run_dynamic_obstacle_ablation() {
  mkdir -p logs
  ros2 run embodied_navigation dynamic_obstacle_model_benchmark \
    --output logs/dynamic_obstacle_model_ablation.json
  python3 tools/evaluation/verify_dynamic_obstacle_ablation.py \
    logs/dynamic_obstacle_model_ablation.json \
    --markdown logs/dynamic_obstacle_model_ablation.md
  ros2 run embodied_navigation dynamic_obstacle_association_benchmark \
    --output logs/dynamic_obstacle_association_ablation.json
  python3 tools/evaluation/verify_dynamic_obstacle_association.py \
    logs/dynamic_obstacle_association_ablation.json \
    --markdown logs/dynamic_obstacle_association_ablation.md
  ros2 run embodied_navigation dynamic_obstacle_uncertainty_benchmark \
    --output logs/dynamic_obstacle_uncertainty_ablation.json
  python3 tools/evaluation/verify_dynamic_obstacle_uncertainty.py \
    logs/dynamic_obstacle_uncertainty_ablation.json \
    --markdown logs/dynamic_obstacle_uncertainty_ablation.md
}

run_base() {
  bash scripts/run_core_tests.sh
  colcon build --symlink-install --allow-overriding \
    embodied_agent_interfaces embodied_agent_core embodied_voice_frontend \
    embodied_agent_cpp embodied_online_agent \
    embodied_offline_agent embodied_simulation embodied_slam embodied_navigation \
    embodied_slam_tools
  colcon test --packages-select \
    embodied_agent_interfaces embodied_agent_core embodied_voice_frontend \
    embodied_agent_cpp embodied_online_agent \
    embodied_offline_agent embodied_simulation embodied_slam embodied_navigation \
    embodied_slam_tools \
    --event-handlers console_direct+
  colcon test-result --verbose
  bash scripts/smoke_test.sh
  bash scripts/smoke_test_online_wake_config.sh
  bash scripts/smoke_test_recognition_retry.sh
  bash scripts/smoke_test_audio_endpoint.sh
  bash scripts/smoke_test_silero_vad_sidecar.sh
  bash scripts/smoke_test_keyword_wake_sidecar.sh
  bash scripts/smoke_test_openwakeword_sidecar.sh
  bash scripts/smoke_test_livekit_wakeword_sidecar.sh
  bash scripts/smoke_test_kws_score_calibration.sh
  bash scripts/smoke_test_voice_readiness.sh
  python3 -m pytest -q tests/integration/voice/test_voice_provider_preflight.py
  bash scripts/smoke_test_lifecycle.sh
  bash scripts/smoke_test_agent_lifecycle.sh online
  bash scripts/smoke_test_agent_lifecycle.sh offline
  bash scripts/smoke_test_typed_action.sh
  bash scripts/smoke_test_typed_action_server.sh
  bash scripts/smoke_test_typed_action_pipeline.sh
  bash scripts/smoke_test_cpp_action_scheduler.sh
  bash scripts/smoke_test_mock_executor.sh
  bash scripts/smoke_test_demo_sequence.sh
  bash scripts/smoke_test_navigation_sequence.sh
  bash scripts/smoke_test_continuous_voice.sh online
  bash scripts/smoke_test_continuous_voice_soak.sh online
  bash scripts/smoke_test_continuous_endpoint_asr.sh online
  bash scripts/smoke_test_continuous_navigation_queue.sh online
  bash scripts/smoke_test_continuous_queue_full.sh online
  bash scripts/smoke_test_continuous_command_ttl.sh online
  bash scripts/smoke_test_continuous_session_timeout.sh online
  bash scripts/smoke_test_continuous_kws_sidecar.sh online
  bash scripts/smoke_test_composed_executor.sh
  bash scripts/smoke_test_namespaced_executor.sh
  bash scripts/smoke_test_offline.sh
  bash scripts/smoke_test_hardware.sh
  bash scripts/smoke_test_simulation.sh
}

run_online() {
  python tests/integration/voice/test_online_api.py
  bash scripts/smoke_test_online_real.sh
}

run_offline() {
  check_offline_runtime
  python3 tools/evaluation/benchmark_offline.py
  third_party/llama.cpp/build/bin/llama-bench \
    -m models/Qwen3-0.6B-Q8_0.gguf -p 64 -n 128 -t "${LLAMA_THREADS:-8}"
  bash tools/evaluation/evaluate_instruction_following.sh
  bash scripts/smoke_test_offline_real.sh
  bash scripts/smoke_test_offline_voice_real.sh
  bash scripts/smoke_test_offline_sherpa_typed_simulation.sh
}

run_gazebo() {
  bash scripts/smoke_test_gazebo.sh
  bash scripts/smoke_test_gazebo_typed_action.sh
}

run_slam_evaluation_stage() {
  local fixture_dir="logs/slam_evaluation_fixture"
  python3 -m pytest -q \
    tests/evaluation/test_slam_trajectory_evaluation.py \
    tests/evaluation/test_slam_evaluation_comparison.py \
    tests/evaluation/test_openloris_groundtruth_setup.py \
    tests/evaluation/test_openloris_rosbag_setup.py \
    tests/evaluation/test_openloris_experiment_manifest.py \
    tests/evaluation/test_slam_degradation_analysis.py \
    tests/evaluation/test_rosbag_trajectory_adapter.py
  python3 tools/evaluation/generate_slam_evaluation_fixture.py --output-dir "$fixture_dir"
  python3 tools/evaluation/evaluate_slam_trajectory.py \
    --reference "$fixture_dir/reference.tum" \
    --estimate "$fixture_dir/dead_reckoning.tum" \
    --output "$fixture_dir/dead_reckoning_report.json"
  python3 tools/evaluation/evaluate_slam_trajectory.py \
    --reference "$fixture_dir/reference.tum" \
    --estimate "$fixture_dir/loop_corrected.tum" \
    --output "$fixture_dir/loop_corrected_report.json" \
    --max-ate-rmse 0.05 \
    --max-rpe-translation-rmse 0.05
  python3 tools/evaluation/compare_slam_evaluations.py \
    --baseline "$fixture_dir/dead_reckoning_report.json" \
    --corrected "$fixture_dir/loop_corrected_report.json" \
    --output "$fixture_dir/comparison.json"
}

check_offline_runtime() {
  require_file models/sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16/encoder-epoch-99-avg-1.int8.onnx
  require_file models/sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16/decoder-epoch-99-avg-1.int8.onnx
  require_file models/sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16/joiner-epoch-99-avg-1.int8.onnx
  require_file models/vits-melo-tts-zh_en/model.onnx
  check_llama_cpp_runtime
}

check_llama_cpp_runtime() {
  require_file models/Qwen3-0.6B-Q8_0.gguf
  require_file third_party/llama.cpp/build/bin/llama-server
}

check_summer_tts_runtime() {
  require_file third_party/SummerTTS/README.md
  require_file third_party/SummerTTS/include/SynthesizerTrn.h
  require_file third_party/SummerTTS/models/single_speaker_fast.bin
  require_file third_party/SummerTTS/build/tts_test
}

run_sherpa_asr_preflight() {
  python3 scripts/sherpa_asr_smoke.py --preflight-only
}

run_sherpa_asr_smoke() {
  python3 scripts/sherpa_asr_smoke.py --expected-substring "星期三"
}

run_isolated_ros_smoke() {
  local domain_id="$1"
  shift
  local exit_code=0
  echo "[nav2-stage] ROS_DOMAIN_ID=$domain_id $*"
  ROS_DOMAIN_ID="$domain_id" "$@" || exit_code=$?
  if (( exit_code != 0 )); then
    echo "[nav2-stage] FAILED exit=$exit_code $*" >&2
    return "$exit_code"
  fi
  # 连续启动多个 ROS graph 时，给 DDS discovery 和进程组清理留出很短缓冲。
  # 这能避免阶段门禁里相邻 smoke 互相看到上一轮残留节点。
  sleep 0.5
  echo "[nav2-stage] PASS $*"
  return 0
}
