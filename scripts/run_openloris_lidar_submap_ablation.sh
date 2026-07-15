#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME_WORKSPACE="${RUNTIME_WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$RUNTIME_WORKSPACE/scripts/activate.sh"
cd "$PROJECT_ROOT"

ROOT="${OPENLORIS_ROOT:-$RUNTIME_WORKSPACE/datasets/openloris}"
LOG_ROOT="${OPENLORIS_LOG_ROOT:-$RUNTIME_WORKSPACE/logs/openloris}"
SEQUENCES="${OPENLORIS_LIDAR_SUBMAP_SEQUENCES:-corridor1-1,corridor1-2}"
BINARY="${LIDAR_SHADOW_SCAN_MATCH_BINARY:-$RUNTIME_WORKSPACE/install/embodied_slam/lib/embodied_slam/lidar_shadow_scan_match}"
TEMPORAL_BINARY="${LIDAR_LOOP_TEMPORAL_REPLAY_BINARY:-$RUNTIME_WORKSPACE/install/embodied_slam/lib/embodied_slam/lidar_loop_temporal_replay}"
if [[ ! -x "$BINARY" ]]; then
  echo "Missing $BINARY; build embodied_slam or set LIDAR_SHADOW_SCAN_MATCH_BINARY" >&2
  exit 2
fi
if [[ ! -x "$TEMPORAL_BINARY" ]]; then
  echo "Missing $TEMPORAL_BINARY; build embodied_slam or set LIDAR_LOOP_TEMPORAL_REPLAY_BINARY" >&2
  exit 2
fi

IFS=',' read -r -a sequence_list <<< "$SEQUENCES"
baseline_args=()
submap_args=()
temporal_args=()
for sequence in "${sequence_list[@]}"; do
  suffix="${sequence//-/_}"
  output_dir="$LOG_ROOT/$sequence/lidar_shadow_matches"
  groundtruth="$ROOT/groundtruth/$sequence/groundtruth.txt"
  candidates="$LOG_ROOT/$sequence/lidar_loop_candidates/candidates.jsonl"
  baseline="$PROJECT_ROOT/docs/evidence/lidar_submap_baseline_${suffix}.json"
  baseline_markdown="$PROJECT_ROOT/docs/evidence/lidar_submap_baseline_${suffix}.md"
  submap="$PROJECT_ROOT/docs/evidence/lidar_submap_matches_${suffix}.json"
  submap_markdown="$PROJECT_ROOT/docs/evidence/lidar_submap_matches_${suffix}.md"
  for required in \
    "$groundtruth" "$candidates" \
    "$output_dir/scan_corpus.txt" "$output_dir/scan_corpus.json" \
    "$output_dir/odometry_priors.txt" "$output_dir/pairs.txt"; do
    if [[ ! -e "$required" ]]; then
      echo "Missing $required; run openloris-lidar-shadow-matches first" >&2
      exit 2
    fi
  done

  # A/B 两侧由同一个 binary、同一 corpus 和同一 pair 文件现场生成，避免跨版本比较。
  "$BINARY" \
    "$output_dir/scan_corpus.txt" \
    "$output_dir/pairs.txt" \
    "$output_dir/submap_baseline_matches.jsonl"
  python3 scripts/evaluate_lidar_shadow_matches.py \
    --matches "$output_dir/submap_baseline_matches.jsonl" \
    --groundtruth "$groundtruth" \
    --sequence "$sequence" \
    --candidates "$candidates" \
    --corpus-metadata "$output_dir/scan_corpus.json" \
    --output "$baseline" \
    --markdown "$baseline_markdown" \
    --minimum-pair-time-coverage 0.85

  # 固定三帧局部窗口和统一 ICP 参数；两个序列不得按真值分别调参。
  "$BINARY" \
    "$output_dir/scan_corpus.txt" \
    "$output_dir/pairs.txt" \
    "$output_dir/submap_matches.jsonl" \
    --point-stride 1 \
    --minimum-points 30 \
    --submap-odometry "$output_dir/odometry_priors.txt" \
    --submap-half-window 1 \
    --submap-max-time-delta 0.75 \
    --submap-point-stride 2 \
    --submap-minimum-scans 2 \
    --submap-minimum-points 60

  python3 scripts/evaluate_lidar_shadow_matches.py \
    --matches "$output_dir/submap_matches.jsonl" \
    --groundtruth "$groundtruth" \
    --sequence "$sequence" \
    --candidates "$candidates" \
    --corpus-metadata "$output_dir/scan_corpus.json" \
    --output "$submap" \
    --markdown "$submap_markdown" \
    --temporal-replay-binary "$TEMPORAL_BINARY" \
    --minimum-pair-time-coverage 0.85
  baseline_args+=(--baseline "$sequence=$baseline")
  submap_args+=(--submap "$sequence=$submap")
  temporal_args+=(--report "$sequence=$submap")
done

python3 scripts/compare_lidar_submap_ablation.py \
  "${baseline_args[@]}" \
  "${submap_args[@]}" \
  --output "$PROJECT_ROOT/docs/evidence/lidar_submap_ablation_multisequence.json" \
  --markdown "$PROJECT_ROOT/docs/evidence/lidar_submap_ablation_multisequence.md"

python3 scripts/compare_lidar_temporal_ablation.py \
  "${temporal_args[@]}" \
  --output "$PROJECT_ROOT/docs/evidence/lidar_temporal_ablation_multisequence.json" \
  --markdown "$PROJECT_ROOT/docs/evidence/lidar_temporal_ablation_multisequence.md"

python3 scripts/compare_lidar_sequence_ablation.py \
  "${temporal_args[@]}" \
  --output "$PROJECT_ROOT/docs/evidence/lidar_sequence_ablation_multisequence.json" \
  --markdown "$PROJECT_ROOT/docs/evidence/lidar_sequence_ablation_multisequence.md"
