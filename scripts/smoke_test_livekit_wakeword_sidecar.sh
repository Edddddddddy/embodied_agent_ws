#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((195 + $$ % 25))}"

FAKE_PACKAGE_DIR="$(mktemp -d)"
LOG_FILE="$(mktemp)"

python3 - "$FAKE_PACKAGE_DIR" <<'PY'
from pathlib import Path
import sys

root = Path(sys.argv[1])
package = root / "livekit" / "wakeword"
package.mkdir(parents=True)
(root / "livekit" / "__init__.py").write_text("", encoding="utf-8")
(package / "__init__.py").write_text(
    """
class WakeWordModel:
    def __init__(self, **kwargs):
        self.kwargs = kwargs

    def predict(self, audio):
        # fake_livekit_wake 始终高于阈值，用于验证 ROS sidecar runtime 链路。
        return {"fake_livekit_wake": 0.93, "background": 0.01}
""".lstrip(),
    encoding="utf-8",
)
PY

PYTHONPATH="$FAKE_PACKAGE_DIR:${PYTHONPATH:-}" \
setsid ros2 run embodied_voice_frontend keyword_wake --ros-args \
  -p mode:=livekit \
  -p provider_name:=livekit_test \
  -p livekit_wakeword_models:="[fake_xiaozhi.onnx]" \
  -p livekit_wakeword_threshold:=0.5 \
  >"$LOG_FILE" 2>&1 &
KWS_PID=$!

cleanup() {
  kill -TERM -- "-$KWS_PID" 2>/dev/null || true
  sleep 0.2
  kill -KILL -- "-$KWS_PID" 2>/dev/null || true
  wait "$KWS_PID" 2>/dev/null || true
  rm -rf "$FAKE_PACKAGE_DIR"
  rm -f "$LOG_FILE"
}
trap cleanup EXIT

if ! timeout 15 python3 "$WORKSPACE/tests/integration/test_livekit_wakeword_sidecar.py"; then
  cat "$LOG_FILE" >&2
  exit 1
fi

echo "PASS: LiveKit WakeWord sidecar consumes audio and publishes wake_event_input"
