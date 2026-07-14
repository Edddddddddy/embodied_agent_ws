#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
cd "$WORKSPACE"

SEQUENCE="${OPENLORIS_SEQUENCE:-corridor1-1}"
OPENLORIS_ROOT="${OPENLORIS_ROOT:-$WORKSPACE/datasets/openloris}"
SOURCE_DIR="${OPENLORIS_OUTPUT_DIR:-$WORKSPACE/logs/openloris/$SEQUENCE}"
GRAPH="${GTSAM_GRAPH_FILE:-$SOURCE_DIR/gtsam_graph.txt}"
REFERENCE="${OPENLORIS_REFERENCE:-$OPENLORIS_ROOT/groundtruth/$SEQUENCE/groundtruth.txt}"
OUTPUT_DIR="${GTSAM_ABLATION_OUTPUT_DIR:-$SOURCE_DIR/robust_kernel_ablation}"
OPTIMIZER="$WORKSPACE/install/embodied_slam/lib/embodied_slam/gtsam_graph_optimize"

if [[ ! -s "$GRAPH" ]]; then
  echo "Missing fixed graph snapshot: $GRAPH" >&2
  echo "Generate it once with: bash scripts/acceptance_test.sh openloris-long-loop-evidence" >&2
  exit 2
fi
if [[ ! -s "$REFERENCE" ]]; then
  echo "Missing OpenLORIS ground truth: $REFERENCE" >&2
  exit 2
fi
if [[ ! -x "$OPTIMIZER" ]]; then
  echo "Missing GTSAM optimizer: $OPTIMIZER" >&2
  echo "Build first: colcon build --packages-select embodied_slam" >&2
  exit 2
fi

python3 scripts/run_gtsam_robust_kernel_ablation.py \
  --graph "$GRAPH" --reference "$REFERENCE" --optimizer "$OPTIMIZER" \
  --output-dir "$OUTPUT_DIR" \
  --loop-id-separation "${GTSAM_LOOP_ID_SEPARATION:-20}" \
  --max-time-diff "${SLAM_MAX_TIME_DIFF_S:-0.05}" \
  --rpe-delta "${SLAM_RPE_DELTA_S:-1.0}" \
  --min-match-ratio "${SLAM_MIN_MATCH_RATIO:-0.80}" \
  --loop-radius "${SLAM_LOOP_RADIUS_M:-1.0}" \
  --loop-yaw-tolerance-deg "${SLAM_LOOP_YAW_TOLERANCE_DEG:-180.0}" \
  --loop-min-separation "${SLAM_LOOP_MIN_SEPARATION_S:-60.0}"
