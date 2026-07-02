#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
LEVEL="${1:-mock}"
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
  bash scripts/smoke_test_typed_action.sh
  bash scripts/smoke_test_offline.sh
  bash scripts/smoke_test_hardware.sh
  bash scripts/smoke_test_simulation.sh
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
  preflight) check_offline_runtime ;;
  mock) run_base ;;
  online) python scripts/test_online_api.py; bash scripts/smoke_test_online_real.sh ;;
  offline) check_offline_runtime; bash scripts/benchmark_offline.sh; bash scripts/evaluate_instruction_following.sh; bash scripts/smoke_test_offline_real.sh; bash scripts/smoke_test_offline_voice_real.sh ;;
  gazebo) bash scripts/smoke_test_gazebo.sh ;;
  gazebo-voice) check_offline_runtime; bash scripts/smoke_test_gazebo_voice.sh ;;
  gazebo-voice-online) check_offline_runtime; bash scripts/smoke_test_gazebo_voice_online.sh ;;
  all) run_base; python scripts/test_online_api.py; bash scripts/smoke_test_online_real.sh; check_offline_runtime; bash scripts/benchmark_offline.sh; bash scripts/evaluate_instruction_following.sh; bash scripts/smoke_test_offline_real.sh; bash scripts/smoke_test_offline_voice_real.sh ;;
  *) echo "Usage: $0 {preflight|mock|online|offline|gazebo|gazebo-voice|gazebo-voice-online|all}" >&2; exit 2 ;;
esac
