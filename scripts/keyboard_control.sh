#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

if [[ ! -t 0 ]]; then
  echo "ERROR: keyboard_control.sh 必须在交互式终端中运行（stdin 不是 TTY）。" >&2
  exit 2
fi

# 两个键盘进程若共用 requester/topic，会让控制权所有者无法区分来源；非阻塞 flock
# 在第二个进程启动时立即报错，避免它在后台持续发布缓存速度。
LOCK_ROOT="${XDG_RUNTIME_DIR:-/tmp}"
LOCK_FILE="${LOCK_ROOT}/embodied-agent-keyboard-${UID}-${ROS_DOMAIN_ID:-0}.lock"
exec 9>"${LOCK_FILE}"
if ! flock --nonblock 9; then
  echo "ERROR: 当前 ROS_DOMAIN_ID 已有 keyboard_control 实例：${LOCK_FILE}" >&2
  exit 3
fi

if [[ -r "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
  echo "ERROR: 未找到 /opt/ros/${ROS_DISTRO}/setup.bash。" >&2
  exit 2
fi

if [[ -r "${ROOT_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/install/setup.bash"
else
  echo "ERROR: 未找到 install/setup.bash；请先执行 colcon build。" >&2
  exit 2
fi

cat <<'EOF'
键盘控制已启动（按住/重复按运动键，松开 600 ms 后自动停止）：
  W/S：前进/后退    A/D：左转/右转
  Space：进入 HOLD   X：锁存急停
  R：急停时第一次解锁到 HOLD；再次按 R 恢复自动控制
  Q：发布零速度后安全退出
EOF

exec ros2 run embodied_agent_cpp keyboard_teleop --ros-args \
  -p linear_speed:="${KEYBOARD_LINEAR_SPEED:-0.20}" \
  -p angular_speed:="${KEYBOARD_ANGULAR_SPEED:-0.80}" \
  -p max_linear_speed:="${KEYBOARD_MAX_LINEAR_SPEED:-0.26}" \
  -p max_angular_speed:="${KEYBOARD_MAX_ANGULAR_SPEED:-1.82}" \
  -p deadman_timeout_ms:="${KEYBOARD_DEADMAN_TIMEOUT_MS:-600}" \
  -p publish_rate_hz:="${KEYBOARD_PUBLISH_RATE_HZ:-20.0}" \
  "$@"
