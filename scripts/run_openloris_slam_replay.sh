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
SEQUENCE="${OPENLORIS_SEQUENCE:-office1-1}"
OUTPUT_DIR="${OPENLORIS_OUTPUT_DIR:-$WORKSPACE/logs/openloris/$SEQUENCE}"
ROOT="${OPENLORIS_ROOT:-$WORKSPACE/datasets/openloris}"
OPENLORIS_BAG="${OPENLORIS_BAG:-$ROOT/rosbag/$SEQUENCE/$SEQUENCE.bag}"
if [[ ! -e "$OPENLORIS_BAG" ]]; then
  echo "Missing $OPENLORIS_BAG" >&2
  echo "Run: bash scripts/acceptance_test.sh openloris-rosbag-setup" >&2
  exit 2
fi
PARAMS_FILE="${OPENLORIS_PARAMS_FILE:-$WORKSPACE/install/embodied_slam/share/embodied_slam/config/openloris_mapping_${BACKEND}.yaml}"
ESTIMATE="$OUTPUT_DIR/${BACKEND}_estimate.tum"
REPORT="$OUTPUT_DIR/${BACKEND}_report.json"
DEGRADATION="$OUTPUT_DIR/${BACKEND}_degradation.json"
LAUNCH_LOG="$OUTPUT_DIR/${BACKEND}_replay.log"
MANIFEST="$OUTPUT_DIR/${BACKEND}_manifest.json"
CONSTRAINT_LOG="$OUTPUT_DIR/${BACKEND}_constraints.jsonl"
LOOP_REPORT="$OUTPUT_DIR/${BACKEND}_loop_constraints.json"
CONTRACT="$OUTPUT_DIR/bag_contract.json"
BAG_SOURCE="${OPENLORIS_BAG_SOURCE:-$(dirname "$OPENLORIS_BAG")/source.json}"
REFERENCE="$ROOT/groundtruth/$SEQUENCE/groundtruth.txt"
mkdir -p "$OUTPUT_DIR"
rm -f "$CONSTRAINT_LOG" "$LOOP_REPORT"

python3 -c "import rosbags" || {
  echo "Missing rosbags; run: pip install -r requirements-slam-eval.txt" >&2
  exit 2
}
ros2 run embodied_slam_tools openloris_rosbag_inspect "$OPENLORIS_BAG" \
  --output "$CONTRACT"
test -s "$PARAMS_FILE"
if [[ ! -s "$BAG_SOURCE" ]]; then
  echo "Missing verified bag provenance: $BAG_SOURCE" >&2
  echo "Prepare the bag with scripts/setup_openloris_rosbag.py before claiming real-data evidence." >&2
  exit 2
fi

echo "[OpenLORIS] sequence=$SEQUENCE backend=$BACKEND bag=$OPENLORIS_BAG"
echo "[OpenLORIS] params=$PARAMS_FILE"
# Fast DDS 的默认端口公式只允许 domain 0..232；使用进程号取模隔离并发验收，
# 同时拒绝外部传入的非法值，避免节点启动前出现含糊的 RTPS 端口错误。
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((120 + $$ % 80))}"
if ! [[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] || (( ROS_DOMAIN_ID > 232 )); then
  echo "Invalid ROS_DOMAIN_ID=$ROS_DOMAIN_ID; expected an integer in [0, 232]" >&2
  exit 2
fi
echo "[OpenLORIS] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
# 回放器统一发布 /clock、隔离后的 TF 和 LaserScan；Ceres/GTSAM 仅替换后端插件，
# 从而保证 A/B 的激光前端、输入消息顺序和评估器完全一致。
SECONDS=0
ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
EMBODIED_SLAM_CONSTRAINT_LOG="$CONSTRAINT_LOG" \
ros2 launch embodied_slam openloris_mapping.launch.py \
  bag_path:="$OPENLORIS_BAG" params_file:="$PARAMS_FILE" \
  replay_rate:="${OPENLORIS_REPLAY_RATE:-1.0}" \
  startup_delay_s:="${OPENLORIS_STARTUP_DELAY_S:-5.0}" \
  output_path:="$ESTIMATE" use_rviz:="${OPENLORIS_USE_RVIZ:-false}" \
  2>&1 | tee "$LAUNCH_LOG"
REPLAY_WALL_CLOCK_S="$SECONDS"
echo "[OpenLORIS] replay_wall_clock_s=$REPLAY_WALL_CLOCK_S"

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
DEGRADATION_ARGS=(
  --reference "$REFERENCE" --estimate "$ESTIMATE"
  --max-time-diff "${SLAM_MAX_TIME_DIFF_S:-0.05}"
  --output "$DEGRADATION"
)
if [[ -n "${OPENLORIS_ANNOTATIONS:-}" ]]; then
  DEGRADATION_ARGS+=(--annotations "$OPENLORIS_ANNOTATIONS")
fi
python3 scripts/analyze_slam_degradation.py "${DEGRADATION_ARGS[@]}"
if [[ "$BACKEND" == "gtsam" && "${OPENLORIS_EVALUATE_LOOP_CONSTRAINTS:-false}" == "true" ]]; then
  if [[ ! -s "$CONSTRAINT_LOG" ]]; then
    echo "FAIL: missing accepted-constraint evidence: $CONSTRAINT_LOG" >&2
    exit 1
  fi
  LOOP_ARGS=(
    --reference "$REFERENCE" --constraints "$CONSTRAINT_LOG" --output "$LOOP_REPORT"
    --timestamp-tolerance "${SLAM_MAX_TIME_DIFF_S:-0.05}"
    --loop-radius "${SLAM_LOOP_RADIUS_M:-0.50}"
    --loop-min-separation "${SLAM_LOOP_MIN_SEPARATION_S:-10.0}"
    --loop-event-gap "${SLAM_LOOP_EVENT_GAP_S:-2.0}"
  )
  if [[ -n "${SLAM_MIN_LOOP_PRECISION:-}" ]]; then
    LOOP_ARGS+=(--min-precision "$SLAM_MIN_LOOP_PRECISION")
  fi
  if [[ -n "${SLAM_MIN_LOOP_EVENT_RECALL:-}" ]]; then
    LOOP_ARGS+=(--min-event-recall "$SLAM_MIN_LOOP_EVENT_RECALL")
  fi
  python3 scripts/evaluate_loop_constraints.py "${LOOP_ARGS[@]}"
fi
LOOP_MANIFEST_ARGS=()
if [[ -s "$LOOP_REPORT" ]]; then
  LOOP_MANIFEST_ARGS+=(--constraint-log "$CONSTRAINT_LOG" --loop-report "$LOOP_REPORT")
fi
ANNOTATION_MANIFEST_ARGS=()
if [[ -n "${OPENLORIS_ANNOTATIONS:-}" ]]; then
  ANNOTATION_MANIFEST_ARGS+=(--annotations "$OPENLORIS_ANNOTATIONS")
fi
python3 scripts/build_openloris_experiment_manifest.py \
  --workspace "$WORKSPACE" --sequence "$SEQUENCE" --backend "$BACKEND" \
  --bag "$OPENLORIS_BAG" --bag-source "$BAG_SOURCE" --contract "$CONTRACT" \
  --params "$PARAMS_FILE" --estimate "$ESTIMATE" --report "$REPORT" \
  --degradation "$DEGRADATION" \
  --launch-log "$LAUNCH_LOG" --replay-rate "${OPENLORIS_REPLAY_RATE:-1.0}" \
  --wall-clock-s "$REPLAY_WALL_CLOCK_S" \
  "${LOOP_MANIFEST_ARGS[@]}" \
  "${ANNOTATION_MANIFEST_ARGS[@]}" \
  --output "$MANIFEST"

echo "PASS: OpenLORIS $BACKEND replay/evaluation"
echo "Evidence: $ESTIMATE $REPORT $DEGRADATION $LAUNCH_LOG $MANIFEST"
if [[ -s "$LOOP_REPORT" ]]; then
  echo "Loop evidence: $CONSTRAINT_LOG $LOOP_REPORT"
fi
