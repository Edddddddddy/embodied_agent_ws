#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
MODEL="${SUMMER_TTS_MODEL:-$WORKSPACE/third_party/SummerTTS/models/single_speaker_fast.bin}"
SERVICE="${SUMMER_TTS_SERVICE_NAME:-/tts/synthesize}"
LOG="$(mktemp)"
PID=""

cleanup() {
  if [[ -n "$PID" ]]; then
    kill "$PID" 2>/dev/null || true
    wait "$PID" 2>/dev/null || true
  fi
  rm -f "$LOG"
}
trap cleanup EXIT

source "$WORKSPACE/scripts/activate.sh"
cd "$WORKSPACE"

if [[ ! -x "$WORKSPACE/install/embodied_agent_cpp/lib/embodied_agent_cpp/summer_tts_service" ]]; then
  colcon build --symlink-install --allow-overriding \
    --packages-select embodied_agent_interfaces embodied_agent_cpp embodied_offline_agent
fi
set +u
source "$WORKSPACE/install/setup.bash"
set -u

ros2 run embodied_agent_cpp summer_tts_service --ros-args \
  -p model_path:="$MODEL" \
  -p service_name:="$SERVICE" >"$LOG" 2>&1 &
PID=$!

for _ in $(seq 1 60); do
  if ros2 service list | grep -Fx "$SERVICE" >/dev/null 2>&1; then
    break
  fi
  if ! kill -0 "$PID" 2>/dev/null; then
    echo "summer_tts_service exited early. Recent log:" >&2
    tail -80 "$LOG" >&2 || true
    exit 1
  fi
  sleep 1
done

if ! ros2 service list | grep -Fx "$SERVICE" >/dev/null 2>&1; then
  echo "summer_tts_service did not expose $SERVICE. Recent log:" >&2
  tail -80 "$LOG" >&2 || true
  exit 1
fi

python3 "$WORKSPACE/scripts/summer_tts_service_probe.py" \
  --service "$SERVICE" \
  --text "${SUMMER_TTS_CACHE_TEXT:-好的。}" \
  --repeat "${SUMMER_TTS_CACHE_REPEAT:-2}" \
  --require-cache-hit
