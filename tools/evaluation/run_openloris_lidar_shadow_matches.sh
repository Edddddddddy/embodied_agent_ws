#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME_WORKSPACE="${RUNTIME_WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$RUNTIME_WORKSPACE/scripts/activate.sh"
cd "$PROJECT_ROOT"

ROOT="${OPENLORIS_ROOT:-$RUNTIME_WORKSPACE/datasets/openloris}"
LOG_ROOT="${OPENLORIS_LOG_ROOT:-$RUNTIME_WORKSPACE/logs/openloris}"
SEQUENCES="${OPENLORIS_LIDAR_SHADOW_SEQUENCES:-corridor1-1,corridor1-2}"
BINARY="${LIDAR_SHADOW_SCAN_MATCH_BINARY:-$RUNTIME_WORKSPACE/install/embodied_slam/lib/embodied_slam/lidar_shadow_scan_match}"
if [[ ! -x "$BINARY" ]]; then
  echo "Missing $BINARY; build embodied_slam or set LIDAR_SHADOW_SCAN_MATCH_BINARY" >&2
  exit 2
fi

IFS=',' read -r -a sequence_list <<< "$SEQUENCES"
aggregate_args=()
for sequence in "${sequence_list[@]}"; do
  graph="$LOG_ROOT/$sequence/gtsam_graph.txt"
  bag="$ROOT/derived/$sequence/$sequence-slam.bag"
  groundtruth="$ROOT/groundtruth/$sequence/groundtruth.txt"
  candidates="$LOG_ROOT/$sequence/lidar_loop_candidates/candidates.jsonl"
  suffix="${sequence//-/_}"
  candidate_evidence="$PROJECT_ROOT/docs/evidence/lidar_loop_candidates_${suffix}.json"
  output_dir="$LOG_ROOT/$sequence/lidar_shadow_matches"
  mkdir -p "$output_dir"
  for required in "$graph" "$bag" "$groundtruth" "$candidates" "$candidate_evidence"; do
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
    --odometry-output "$output_dir/odometry_priors.txt" \
    --sample-interval 0.5
  python3 tools/evaluation/prepare_lidar_shadow_pairs.py \
    --candidates "$candidates" \
    --odometry-priors "$output_dir/odometry_priors.txt" \
    --corpus-metadata "$output_dir/scan_corpus.json" \
    --candidate-evidence "$candidate_evidence" \
    --output "$output_dir/pairs.txt" \
    --top-k 10
  "$BINARY" \
    "$output_dir/scan_corpus.txt" \
    "$output_dir/pairs.txt" \
    "$output_dir/matches.jsonl"

  published_json="$PROJECT_ROOT/docs/evidence/lidar_shadow_matches_${suffix}.json"
  published_markdown="$PROJECT_ROOT/docs/evidence/lidar_shadow_matches_${suffix}.md"
  python3 tools/evaluation/evaluate_lidar_shadow_matches.py \
    --matches "$output_dir/matches.jsonl" \
    --groundtruth "$groundtruth" \
    --sequence "$sequence" \
    --candidates "$candidates" \
    --corpus-metadata "$output_dir/scan_corpus.json" \
    --output "$published_json" \
    --markdown "$published_markdown" \
    --minimum-pair-time-coverage 0.85
  aggregate_args+=(--report "$sequence=$published_json")
done

python3 tools/evaluation/compare_lidar_shadow_match_sequences.py \
  "${aggregate_args[@]}" \
  --output "$PROJECT_ROOT/docs/evidence/lidar_shadow_matches_multisequence.json" \
  --markdown "$PROJECT_ROOT/docs/evidence/lidar_shadow_matches_multisequence.md"
