#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
cd "$WORKSPACE"

BACKEND="${1:-ceres}"
if [[ "$BACKEND" != "ceres" && "$BACKEND" != "gtsam" ]]; then
  echo "Usage: OPENLORIS_BAG=/path/to/sequence.bag $0 {ceres|gtsam}" >&2
  exit 2
fi
if [[ -z "${OPENLORIS_BAG:-}" || ! -e "$OPENLORIS_BAG" ]]; then
  echo "Set OPENLORIS_BAG to an extracted OpenLORIS ROS 1 .bag file." >&2
  exit 2
fi

SEQUENCE="${OPENLORIS_SEQUENCE:-office1-1}"
OUTPUT_DIR="${OPENLORIS_OUTPUT_DIR:-$WORKSPACE/logs/openloris/$SEQUENCE}"
ROOT="${OPENLORIS_ROOT:-$WORKSPACE/datasets/openloris}"
PARAMS_FILE="$WORKSPACE/install/embodied_slam/share/embodied_slam/config/openloris_mapping_${BACKEND}.yaml"
ESTIMATE="$OUTPUT_DIR/${BACKEND}_estimate.tum"
REPORT="$OUTPUT_DIR/${BACKEND}_report.json"
LAUNCH_LOG="$OUTPUT_DIR/${BACKEND}_replay.log"
REFERENCE="$ROOT/groundtruth/$SEQUENCE/groundtruth.txt"
mkdir -p "$OUTPUT_DIR"

python3 -c "import rosbags" || {
  echo "Missing rosbags; run: pip install -r requirements-slam-eval.txt" >&2
  exit 2
}
ros2 run embodied_slam_tools openloris_rosbag_inspect "$OPENLORIS_BAG" \
  --output "$OUTPUT_DIR/bag_contract.json"
test -s "$PARAMS_FILE"

echo "[OpenLORIS] sequence=$SEQUENCE backend=$BACKEND bag=$OPENLORIS_BAG"
# 回放器统一发布 /clock、隔离后的 TF 和 LaserScan；Ceres/GTSAM 仅替换后端插件，
# 从而保证 A/B 的激光前端、输入消息顺序和评估器完全一致。
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-241}" ros2 launch embodied_slam openloris_mapping.launch.py \
  bag_path:="$OPENLORIS_BAG" params_file:="$PARAMS_FILE" \
  replay_rate:="${OPENLORIS_REPLAY_RATE:-1.0}" \
  startup_delay_s:="${OPENLORIS_STARTUP_DELAY_S:-5.0}" \
  output_path:="$ESTIMATE" use_rviz:="${OPENLORIS_USE_RVIZ:-false}" \
  2>&1 | tee "$LAUNCH_LOG"

pose_count="$(grep -cv '^#' "$ESTIMATE")"
if (( pose_count < ${OPENLORIS_MIN_POSES:-100} )); then
  echo "FAIL: only $pose_count estimated poses; inspect $LAUNCH_LOG" >&2
  exit 1
fi
if [[ ! -s "$REFERENCE" ]]; then
  python3 scripts/setup_openloris_groundtruth.py --sequence "$SEQUENCE" --output-root "$ROOT"
fi

EVAL_ARGS=(
  --reference "$REFERENCE"
  --estimate "$ESTIMATE"
  --output "$REPORT"
  --max-time-diff "${SLAM_MAX_TIME_DIFF_S:-0.05}"
  --rpe-delta "${SLAM_RPE_DELTA_S:-1.0}"
  --min-match-ratio "${SLAM_MIN_MATCH_RATIO:-0.80}"
)
if [[ -n "${SLAM_MAX_ATE_RMSE_M:-}" ]]; then
  EVAL_ARGS+=(--max-ate-rmse "$SLAM_MAX_ATE_RMSE_M")
fi
if [[ -n "${SLAM_MAX_RPE_RMSE_M:-}" ]]; then
  EVAL_ARGS+=(--max-rpe-translation-rmse "$SLAM_MAX_RPE_RMSE_M")
fi
python3 scripts/evaluate_slam_trajectory.py "${EVAL_ARGS[@]}"

echo "PASS: OpenLORIS $BACKEND replay/evaluation"
echo "Evidence: $ESTIMATE $REPORT $LAUNCH_LOG"
