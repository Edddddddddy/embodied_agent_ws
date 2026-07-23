#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
cd "$WORKSPACE"

ROOT="${OPENLORIS_ROOT:-$WORKSPACE/datasets/openloris}"
LOG_ROOT="${OPENLORIS_LOG_ROOT:-$WORKSPACE/logs/openloris}"
SEQUENCES="${OPENLORIS_LIDAR_LOOP_SEQUENCES:-corridor1-1,corridor1-2}"
BINARY="${LIDAR_LOOP_CANDIDATE_BINARY:-$WORKSPACE/install/embodied_slam/lib/embodied_slam/lidar_loop_candidates}"
if [[ ! -x "$BINARY" ]]; then
  echo "Missing $BINARY; build embodied_slam or set LIDAR_LOOP_CANDIDATE_BINARY" >&2
  exit 2
fi

IFS=',' read -r -a sequence_list <<< "$SEQUENCES"
aggregate_args=()
for sequence in "${sequence_list[@]}"; do
  graph="$LOG_ROOT/$sequence/gtsam_graph.txt"
  bag="$ROOT/derived/$sequence/$sequence-slam.bag"
  groundtruth="$ROOT/groundtruth/$sequence/groundtruth.txt"
  output_dir="$LOG_ROOT/$sequence/lidar_loop_candidates"
  mkdir -p "$output_dir"
  for required in "$graph" "$bag" "$groundtruth"; do
    if [[ ! -e "$required" ]]; then
      echo "Missing required evidence input: $required" >&2
      exit 2
    fi
  done

  python3 tools/evaluation/extract_openloris_scan_corpus.py \
    --graph "$graph" \
    --bag "$bag" \
    --output "$output_dir/scan_corpus.txt" \
    --metadata "$output_dir/scan_corpus.json" \
    --sample-interval 0.5

  "$BINARY" \
    "$output_dir/scan_corpus.txt" \
    "$output_dir/candidates.jsonl" \
    20 60 20.0 30 60.0 60 60 0.0

  suffix="${sequence//-/_}"
  published_json="$WORKSPACE/docs/evidence/slam/lidar_loop_candidates_${suffix}.json"
  published_markdown="$WORKSPACE/docs/evidence/slam/lidar_loop_candidates_${suffix}.md"
  python3 tools/evaluation/evaluate_lidar_loop_candidates.py \
    --candidates "$output_dir/candidates.jsonl" \
    --groundtruth "$groundtruth" \
    --corpus-metadata "$output_dir/scan_corpus.json" \
    --sequence "$sequence" \
    --output "$published_json" \
    --markdown "$published_markdown"
  aggregate_args+=(--report "$sequence=$published_json")
done

python3 tools/evaluation/compare_lidar_loop_candidate_sequences.py \
  "${aggregate_args[@]}" \
  --output "$WORKSPACE/docs/evidence/slam/lidar_loop_candidates_multisequence.json" \
  --markdown "$WORKSPACE/docs/evidence/slam/lidar_loop_candidates_multisequence.md"
