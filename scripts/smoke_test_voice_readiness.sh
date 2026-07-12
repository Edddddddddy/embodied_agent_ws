#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((210 + $$ % 15))}"

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
from embodied_agent_core.ros_qos import audio_qos, state_qos
from embodied_agent_interfaces.msg import AudioFrontendStatus
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

rclpy.init()
node = Node("voice_readiness_probe_publisher")
audio_pub = node.create_publisher(
    AudioFrontendStatus, "/audio/frontend_metrics", state_qos()
)
pcm_pub = node.create_publisher(UInt8MultiArray, "/audio/clean_pcm", audio_qos())
deadline = time.monotonic() + 3.0
metrics = AudioFrontendStatus()
metrics.rms = 0.025
metrics.peak = 1200
metrics.speech = True
while time.monotonic() < deadline:
    audio_pub.publish(metrics)
    pcm_pub.publish(UInt8MultiArray(data=[1, 0, 2, 0]))
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

OUTPUT="$(python3 "$WORKSPACE/scripts/voice_control_readiness_check.py" --duration 1.5 --require-kws --json)"

python3 - "$OUTPUT" <<'PY'
import json
import sys

payload = json.loads(sys.argv[1])
if not payload["ok"]:
    raise SystemExit(f"readiness should pass: {payload}")
if payload["audio"]["sample_count"] <= 0:
    raise SystemExit(f"missing audio samples: {payload}")
if payload["kws"]["sample_count"] <= 0:
    raise SystemExit(f"missing kws samples: {payload}")
print(json.dumps(payload, ensure_ascii=False, indent=2))
PY

echo "PASS: voice readiness check combines audio and KWS diagnostics"
