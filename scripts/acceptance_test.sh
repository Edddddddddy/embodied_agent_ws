#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
LEVEL="${1:-mock}"

usage() {
  cat <<'EOF'
Usage: acceptance_test.sh MODE

Automated modes:
  core                Typical developer gate: repository, Python unit, C++ unit tests
  preflight           Check offline model/runtime files
  mock                Build, unit tests, and dependency-free ROS smokes
  online              Minimal-token live ASR/LLM/TTS verification
  offline             Real ZipFormer/llama.cpp/Sherpa-TTS verification
  demo                Rich mock demo: ordered actions, accessories, and arc motion
  navigation-demo     Voice-style target navigation and multi-waypoint patrol smoke
  nav2-bridge         Voice navigation commands are converted to Nav2 action goals
  nav2-preflight      Check Nav2/TurtleBot3 voice launch dependencies and arguments
  nav2-turtlebot3     Heavy Gazebo/Nav2 run: voice text drives target navigation/patrol
  continuous-mock     One wake word, several queued commands, and sleep gate
  continuous-soak     Long wake session keeps accepting many queued commands
  continuous-endpoint Endpoint speech_ended commits feed continuous ASR commands
  continuous-multi-command NLU parses one ASR final into ordered queued commands
  continuous-queue-full Busy continuous queue rejects excess commands with feedback
  continuous-ttl      Busy continuous queue expires stale non-priority commands
  continuous-timeout  Voice session timeout requires a fresh wake word
  continuous-kws-mock KWS sidecar opens a continuous session and executes a command
  vad-sidecar         Dependency-free Silero VAD sidecar seam smoke test
  kws-sidecar         Dependency-free keyword wake sidecar seam smoke test
  openwakeword-sidecar Dependency-free openWakeWord adapter runtime smoke test
  livekit-sidecar     Dependency-free LiveKit WakeWord adapter runtime smoke test
  kws-calibration     Dependency-free KWS score calibration smoke test
  voice-readiness     Dependency-free voice readiness smoke test
  provider-preflight  Dependency-free optional VAD/KWS provider preflight
  gazebo              Legacy and typed Action physical motion verification
  gazebo-voice        Offline synthesized speech through typed Action to Gazebo
  gazebo-voice-online Online voice provider through typed Action to Gazebo
  all                 Run all automated release gates; excludes interactive microphone

Interactive modes:
  microphone-offline  Speak into the microphone using the offline Agent
  microphone-online   Speak into the microphone using the online Agent
  continuous-offline  Long-running microphone control using the offline Agent
  continuous-online   Long-running microphone control using the online Agent
  continuous-nav2-offline  Long-running microphone target navigation with Nav2/TurtleBot3
  continuous-nav2-online   Long-running microphone target navigation with Nav2/TurtleBot3
  continuous-live-check {offline|online}  Observe a running live microphone demo and score evidence
  continuous-nav2-live-check {offline|online}  Score a running live Nav2 microphone demo
EOF
}

if [[ "$LEVEL" == "help" || "$LEVEL" == "--help" || "$LEVEL" == "-h" ]]; then
  usage
  exit 0
fi

source "$WORKSPACE/scripts/activate.sh"
cd "$WORKSPACE"

require_file() {
  if [[ ! -s "$1" ]]; then
    echo "MISSING: $1" >&2
    return 1
  fi
  echo "READY: $1"
}

run_base() {
  bash scripts/run_core_tests.sh
  colcon build --symlink-install --allow-overriding \
    embodied_agent_interfaces embodied_agent_cpp embodied_online_agent \
    embodied_offline_agent embodied_simulation
  colcon test --packages-select \
    embodied_agent_interfaces embodied_agent_cpp embodied_online_agent \
    embodied_offline_agent embodied_simulation \
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
  pytest -q tests/integration/test_voice_provider_preflight.py
  bash scripts/smoke_test_lifecycle.sh
  bash scripts/smoke_test_typed_action.sh
  bash scripts/smoke_test_typed_action_server.sh
  bash scripts/smoke_test_typed_action_pipeline.sh
  bash scripts/smoke_test_mock_executor.sh
  bash scripts/smoke_test_demo_sequence.sh
  bash scripts/smoke_test_navigation_sequence.sh
  bash scripts/smoke_test_continuous_voice.sh online
  bash scripts/smoke_test_continuous_voice_soak.sh online
  bash scripts/smoke_test_continuous_endpoint_asr.sh online
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
  python tests/integration/test_online_api.py
  bash scripts/smoke_test_online_real.sh
}

run_offline() {
  check_offline_runtime
  bash scripts/benchmark_offline.sh
  bash scripts/evaluate_instruction_following.sh
  bash scripts/smoke_test_offline_real.sh
  bash scripts/smoke_test_offline_voice_real.sh
}

run_gazebo() {
  bash scripts/smoke_test_gazebo.sh
  bash scripts/smoke_test_gazebo_typed_action.sh
}

check_offline_runtime() {
  require_file models/sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16/encoder-epoch-99-avg-1.int8.onnx
  require_file models/sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16/decoder-epoch-99-avg-1.int8.onnx
  require_file models/sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16/joiner-epoch-99-avg-1.int8.onnx
  require_file models/vits-melo-tts-zh_en/model.onnx
  require_file models/Qwen3-0.6B-Q8_0.gguf
  require_file third_party/llama.cpp/build/bin/llama-server
}

case "$LEVEL" in
  core) bash scripts/run_core_tests.sh ;;
  preflight) check_offline_runtime ;;
  mock) run_base ;;
  online) run_online ;;
  offline) run_offline ;;
  demo) bash scripts/smoke_test_demo_sequence.sh ;;
  navigation-demo) bash scripts/smoke_test_navigation_sequence.sh online; bash scripts/smoke_test_navigation_sequence.sh offline ;;
  nav2-bridge) bash scripts/smoke_test_nav2_bridge.sh ;;
  nav2-preflight) bash scripts/smoke_test_nav2_preflight.sh ;;
  nav2-turtlebot3) bash scripts/smoke_test_nav2_turtlebot3_voice.sh ;;
  continuous-mock) bash scripts/smoke_test_continuous_voice.sh online; bash scripts/smoke_test_continuous_voice.sh offline ;;
  continuous-soak) bash scripts/smoke_test_continuous_voice_soak.sh online; bash scripts/smoke_test_continuous_voice_soak.sh offline ;;
  continuous-endpoint) bash scripts/smoke_test_continuous_endpoint_asr.sh online; bash scripts/smoke_test_continuous_endpoint_asr.sh offline ;;
  continuous-multi-command) bash scripts/smoke_test_continuous_multi_command.sh online; bash scripts/smoke_test_continuous_multi_command.sh offline ;;
  continuous-queue-full) bash scripts/smoke_test_continuous_queue_full.sh online; bash scripts/smoke_test_continuous_queue_full.sh offline ;;
  continuous-ttl) bash scripts/smoke_test_continuous_command_ttl.sh online; bash scripts/smoke_test_continuous_command_ttl.sh offline ;;
  continuous-timeout) bash scripts/smoke_test_continuous_session_timeout.sh online; bash scripts/smoke_test_continuous_session_timeout.sh offline ;;
  continuous-kws-mock) bash scripts/smoke_test_continuous_kws_sidecar.sh online; bash scripts/smoke_test_continuous_kws_sidecar.sh offline ;;
  vad-sidecar) bash scripts/smoke_test_silero_vad_sidecar.sh ;;
  kws-sidecar) bash scripts/smoke_test_keyword_wake_sidecar.sh ;;
  openwakeword-sidecar) bash scripts/smoke_test_openwakeword_sidecar.sh ;;
  livekit-sidecar) bash scripts/smoke_test_livekit_wakeword_sidecar.sh ;;
  kws-calibration) bash scripts/smoke_test_kws_score_calibration.sh ;;
  voice-readiness) bash scripts/smoke_test_voice_readiness.sh ;;
  provider-preflight) pytest -q tests/integration/test_voice_provider_preflight.py ;;
  gazebo) run_gazebo ;;
  gazebo-voice) check_offline_runtime; USE_TYPED_ACTIONS=true bash scripts/smoke_test_gazebo_voice.sh ;;
  gazebo-voice-online) bash scripts/smoke_test_gazebo_voice_online.sh ;;
  microphone-offline) bash scripts/accept_voice_simulation_microphone.sh offline ;;
  microphone-online) bash scripts/accept_voice_simulation_microphone.sh online ;;
  continuous-offline) bash scripts/continuous_voice_control.sh offline ;;
  continuous-online) bash scripts/continuous_voice_control.sh online ;;
  continuous-nav2-offline) bash scripts/continuous_nav2_voice_control.sh offline ;;
  continuous-nav2-online) bash scripts/continuous_nav2_voice_control.sh online ;;
  continuous-live-check)
    CHECK_MODE="${2:-offline}"
    if [[ "$CHECK_MODE" != "offline" && "$CHECK_MODE" != "online" ]]; then
      echo "Usage: $0 continuous-live-check {offline|online}" >&2
      exit 2
    fi
    echo "continuous-live-check=$CHECK_MODE：请先在另一个终端启动 acceptance_test.sh continuous-$CHECK_MODE"
    python3 scripts/continuous_live_check.py --duration "${CONTINUOUS_LIVE_CHECK_DURATION:-180}"
    ;;
  continuous-nav2-live-check)
    CHECK_MODE="${2:-offline}"
    if [[ "$CHECK_MODE" != "offline" && "$CHECK_MODE" != "online" ]]; then
      echo "Usage: $0 continuous-nav2-live-check {offline|online}" >&2
      exit 2
    fi
    echo "continuous-nav2-live-check=$CHECK_MODE：请先在另一个终端启动 acceptance_test.sh continuous-nav2-$CHECK_MODE"
    python3 scripts/continuous_live_check.py \
      --duration "${CONTINUOUS_LIVE_CHECK_DURATION:-240}" \
      --min-asr "${CONTINUOUS_NAV2_LIVE_MIN_ASR:-4}" \
      --min-candidates "${CONTINUOUS_NAV2_LIVE_MIN_CANDIDATES:-2}" \
      --min-success "${CONTINUOUS_NAV2_LIVE_MIN_SUCCESS:-2}"
    ;;
  all) run_base; run_online; run_offline; bash scripts/smoke_test_demo_sequence.sh; run_gazebo; USE_TYPED_ACTIONS=true bash scripts/smoke_test_gazebo_voice.sh; bash scripts/smoke_test_gazebo_voice_online.sh ;;
  *) usage >&2; exit 2 ;;
esac
