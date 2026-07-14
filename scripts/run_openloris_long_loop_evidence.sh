#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
cd "$WORKSPACE"

ROOT="${OPENLORIS_ROOT:-$WORKSPACE/datasets/openloris}"
SEQUENCE="${OPENLORIS_SEQUENCE:-corridor1-1}"
case "$SEQUENCE" in
  corridor1-1|corridor1-2) ;;
  *)
    echo "Pinned 2D long-loop stage supports corridor1-1 and corridor1-2." >&2
    exit 2
    ;;
esac

RANKING_REPORT="${OPENLORIS_RANKING_REPORT:-$WORKSPACE/logs/openloris/revisit_sequence_ranking.json}"
RAW_BAG="$ROOT/rosbag/$SEQUENCE/$SEQUENCE.bag"
RAW_SOURCE="$(dirname "$RAW_BAG")/source.json"
DERIVED_ROOT="$ROOT/derived/$SEQUENCE"
DERIVED_BAG="$DERIVED_ROOT/$SEQUENCE-slam.bag"
DERIVED_SOURCE="$DERIVED_ROOT/source.json"
OUTPUT_DIR="${OPENLORIS_OUTPUT_DIR:-$WORKSPACE/logs/openloris/$SEQUENCE}"
DEFAULT_ANNOTATIONS=""
if [[ "$SEQUENCE" == "corridor1-1" ]]; then
  DEFAULT_ANNOTATIONS="$WORKSPACE/src/embodied_slam/config/openloris_corridor1_1_annotations.json"
fi
ANNOTATIONS="${OPENLORIS_ANNOTATIONS:-$DEFAULT_ANNOTATIONS}"
DEFAULT_MIN_GRAPH_NODES=1000
if [[ "$SEQUENCE" == "corridor1-2" ]]; then
  DEFAULT_MIN_GRAPH_NODES=500
fi

# 先用小型真值包证明序列满足长距离、长时长和空间回访门槛，再下载大型 bag。
# corridor 是反向穿越同一地点，因此 360° LiDAR 使用位置回访口径，而不是相机视角口径。
python3 scripts/setup_openloris_groundtruth.py \
  --sequence "$SEQUENCE" --output-root "$ROOT"
python3 scripts/rank_openloris_revisit_sequences.py \
  --archive "$ROOT/archive/groundtruth.zip" \
  --sensor-contracts "$WORKSPACE/src/embodied_slam/config/openloris_sensor_contracts.json" \
  --output "$RANKING_REPORT"
python3 - "$RANKING_REPORT" "$SEQUENCE" <<'PY'
import json
import sys

report = json.load(open(sys.argv[1], encoding="utf-8"))
row = next(
    (item for item in report.get("ranked_sequences", []) if item.get("sequence") == sys.argv[2]),
    None,
)
if row is None:
    raise SystemExit(f"sequence missing from ground-truth ranking: {sys.argv[2]}")
contract = row.get("sensor_contract", {})
events = int(row.get("position_only_360_lidar", {}).get("event_count", 0))
trajectory = row.get("trajectory", {})
checks = {
    "verified_compatible": contract.get("status") == "verified_compatible",
    "has_revisit_event": events >= 1,
    "path_at_least_100m": float(trajectory.get("path_length_m", 0.0)) >= 100.0,
    "duration_at_least_90s": float(trajectory.get("duration_s", 0.0)) >= 90.0,
}
if not all(checks.values()):
    raise SystemExit(f"sequence is not eligible for validation: {checks}")
# 推荐序列用于首选基准；第二序列只需满足同一传感器与回访契约，避免把“排名第二”误判为无效。
print(f"PASS: selected compatible long-loop validation sequence {sys.argv[2]}")
PY

if [[ ! -s "$RAW_BAG" ]]; then
  python3 scripts/setup_openloris_rosbag.py \
    --sequence "$SEQUENCE" --output-root "$ROOT" --range-only \
    --download-connections "${OPENLORIS_DOWNLOAD_CONNECTIONS:-12}"
fi

if [[ ! -s "$DERIVED_BAG" ]]; then
  mkdir -p "$DERIVED_ROOT"
  COMPACT_ARGS=(
    --input "$RAW_BAG" --source "$RAW_SOURCE"
    --output "$DERIVED_BAG" --output-source "$DERIVED_SOURCE"
    --workspace "$WORKSPACE"
  )
  if [[ "${OPENLORIS_VERIFY_RAW_HASH:-true}" == "true" ]]; then
    COMPACT_ARGS+=(--verify-source-hash)
  fi
  python3 scripts/compact_openloris_rosbag.py "${COMPACT_ARGS[@]}"
fi

colcon build --packages-up-to embodied_slam embodied_slam_tools --symlink-install
OPENLORIS_SEQUENCE="$SEQUENCE" \
OPENLORIS_ROOT="$ROOT" \
OPENLORIS_BAG="$DERIVED_BAG" \
OPENLORIS_BAG_SOURCE="$DERIVED_SOURCE" \
OPENLORIS_OUTPUT_DIR="$OUTPUT_DIR" \
OPENLORIS_ANNOTATIONS="$ANNOTATIONS" \
OPENLORIS_REPLAY_RATE="${OPENLORIS_REPLAY_RATE:-2.0}" \
OPENLORIS_MIN_POSES="${OPENLORIS_MIN_POSES:-250}" \
OPENLORIS_MIN_GRAPH_NODES="${OPENLORIS_MIN_GRAPH_NODES:-$DEFAULT_MIN_GRAPH_NODES}" \
OPENLORIS_EVALUATE_LOOP_CONSTRAINTS=true \
OPENLORIS_EVALUATE_FRONTEND=true \
SLAM_LOOP_RADIUS_M="${SLAM_LOOP_RADIUS_M:-1.0}" \
SLAM_LOOP_YAW_TOLERANCE_DEG="${SLAM_LOOP_YAW_TOLERANCE_DEG:-180.0}" \
SLAM_LOOP_MIN_SEPARATION_S="${SLAM_LOOP_MIN_SEPARATION_S:-60.0}" \
bash scripts/run_openloris_slam_replay.sh gtsam

echo "PASS: OpenLORIS $SEQUENCE long-loop evidence"
echo "Evidence: $RANKING_REPORT $OUTPUT_DIR/gtsam_manifest.json"
