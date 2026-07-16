#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
cd "$WORKSPACE"

SEQUENCE="${OPENLORIS_SEQUENCE:-corridor1-1}"
OPENLORIS_ROOT="${OPENLORIS_ROOT:-$WORKSPACE/datasets/openloris}"
SOURCE_DIR="${OPENLORIS_OUTPUT_DIR:-$WORKSPACE/logs/openloris/$SEQUENCE}"
SOURCE_GRAPH="${GTSAM_GRAPH_FILE:-$SOURCE_DIR/gtsam_graph.txt}"
BAG="${OPENLORIS_DERIVED_BAG:-$OPENLORIS_ROOT/derived/$SEQUENCE/$SEQUENCE-slam.bag}"
REFERENCE="${OPENLORIS_REFERENCE:-$OPENLORIS_ROOT/groundtruth/$SEQUENCE/groundtruth.txt}"
OUTPUT_DIR="${GTSAM_SCAN_OVERLAP_OUTPUT_DIR:-$SOURCE_DIR/scan_overlap_ablation}"
ENRICHED_GRAPH="$OUTPUT_DIR/gtsam_graph_with_scan_overlap.txt"
OPTIMIZER="$WORKSPACE/install/embodied_slam/lib/embodied_slam/gtsam_graph_optimize"
PUBLISHED_SUFFIX="${SEQUENCE//-/_}"
if [[ "$SEQUENCE" == "corridor1-1" ]]; then
  PUBLISHED_JSON="${GTSAM_SCAN_OVERLAP_PUBLISHED_JSON:-$WORKSPACE/docs/evidence/gtsam_scan_overlap_ablation.json}"
  PUBLISHED_MARKDOWN="${GTSAM_SCAN_OVERLAP_PUBLISHED_MARKDOWN:-$WORKSPACE/docs/evidence/gtsam_scan_overlap_ablation.md}"
else
  PUBLISHED_JSON="${GTSAM_SCAN_OVERLAP_PUBLISHED_JSON:-$WORKSPACE/docs/evidence/gtsam_scan_overlap_ablation_${PUBLISHED_SUFFIX}.json}"
  PUBLISHED_MARKDOWN="${GTSAM_SCAN_OVERLAP_PUBLISHED_MARKDOWN:-$WORKSPACE/docs/evidence/gtsam_scan_overlap_ablation_${PUBLISHED_SUFFIX}.md}"
fi

for required in "$SOURCE_GRAPH" "$BAG" "$REFERENCE" "$OPTIMIZER"; do
  if [[ ! -s "$required" ]]; then
    echo "Missing scan-overlap ablation input: $required" >&2
    exit 2
  fi
done

python3 tools/evaluation/augment_pose_graph_scan_overlap.py \
  --graph "$SOURCE_GRAPH" --bag "$BAG" --output "$ENRICHED_GRAPH" \
  --metadata "$OUTPUT_DIR/augmentation.json"

python3 tools/evaluation/run_gtsam_scan_overlap_ablation.py \
  --sequence "$SEQUENCE" \
  --graph "$ENRICHED_GRAPH" --reference "$REFERENCE" --optimizer "$OPTIMIZER" \
  --output-dir "$OUTPUT_DIR" \
  --minimum-overlap "${GTSAM_MIN_SCAN_OVERLAP_RATIO:-0.65}" \
  --minimum-overlap-innovation "${GTSAM_SCAN_OVERLAP_MIN_INNOVATION_M:-1.0}" \
  --augmentation-metadata "$OUTPUT_DIR/augmentation.json" \
  --published-json "$PUBLISHED_JSON" \
  --published-markdown "$PUBLISHED_MARKDOWN"
