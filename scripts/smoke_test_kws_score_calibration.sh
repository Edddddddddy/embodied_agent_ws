#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((205 + $$ % 20))}"

FAKE_PACKAGE_DIR="$(mktemp -d)"
KWS_LOG="$(mktemp)"
PUBLISHER_LOG="$(mktemp)"

python3 - "$FAKE_PACKAGE_DIR" <<'PY'
from pathlib import Path
import sys

root = Path(sys.argv[1])
package = root / "openwakeword"
package.mkdir(parents=True)
(package / "__init__.py").write_text("", encoding="utf-8")
(package / "model.py").write_text(
    """
class Model:
    def __init__(self, **kwargs):
        self.kwargs = kwargs

    def predict(self, audio):
        return {"fake_wake": 0.91, "noise": 0.01}
""".lstrip(),
    encoding="utf-8",
)
PY

PYTHONPATH="$FAKE_PACKAGE_DIR:${PYTHONPATH:-}" \
setsid ros2 run embodied_voice_frontend keyword_wake --ros-args \
  -p mode:=openwakeword \
  -p provider_name:=openwakeword_test \
  -p openwakeword_threshold:=0.5 \
  -p score_publish_period_s:=0.05 \
  >"$KWS_LOG" 2>&1 &
KWS_PID=$!

python3 - <<'PY' >"$PUBLISHER_LOG" 2>&1 &
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

rclpy.init()
node = Node("kws_score_audio_publisher")
publisher = node.create_publisher(UInt8MultiArray, "/audio/clean_pcm", 10)
deadline = time.monotonic() + 3.0
while time.monotonic() < deadline:
    publisher.publish(UInt8MultiArray(data=[1, 0, 2, 0]))
    rclpy.spin_once(node, timeout_sec=0.02)
    time.sleep(0.03)
node.destroy_node()
rclpy.shutdown()
PY
PUBLISHER_PID=$!

cleanup() {
  kill -TERM -- "-$KWS_PID" 2>/dev/null || true
  kill "$PUBLISHER_PID" 2>/dev/null || true
  sleep 0.2
  kill -KILL -- "-$KWS_PID" 2>/dev/null || true
  wait "$KWS_PID" "$PUBLISHER_PID" 2>/dev/null || true
  rm -rf "$FAKE_PACKAGE_DIR"
  rm -f "$KWS_LOG" "$PUBLISHER_LOG"
}
trap cleanup EXIT

OUTPUT="$(python3 "$WORKSPACE/scripts/kws_score_calibration.py" --duration 1.5 --json)"

python3 - "$OUTPUT" <<'PY'
import json
import sys

payload = json.loads(sys.argv[1])
if payload["sample_count"] <= 0:
    raise SystemExit("no KWS score samples collected")
if payload["provider"] != "openwakeword_test":
    raise SystemExit(f"unexpected provider: {payload}")
if payload["max_top_score"] < 0.9:
    raise SystemExit(f"unexpected max score: {payload}")
if payload["suggested_threshold"] < payload["current_threshold"]:
    raise SystemExit(f"unexpected suggested threshold: {payload}")
if "threshold_may_be_too_low_or_environment_noisy" not in payload["warnings"]:
    raise SystemExit(f"expected noisy/low-threshold warning: {payload}")
print(json.dumps(payload, ensure_ascii=False, indent=2))
PY

echo "PASS: KWS score calibration collects /agent/kws_score and suggests a threshold"
