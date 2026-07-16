#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
cd "$WORKSPACE"

SEQUENCES_CSV="${OPENLORIS_SCAN_OVERLAP_SEQUENCES:-corridor1-1,corridor1-2}"
IFS=',' read -r -a SEQUENCES <<< "$SEQUENCES_CSV"
if (( ${#SEQUENCES[@]} < 2 )); then
  echo "Multi-sequence validation requires at least two sequences." >&2
  exit 2
fi

REPORT_ARGS=()
for sequence in "${SEQUENCES[@]}"; do
  report="$WORKSPACE/logs/openloris/$sequence/scan_overlap_ablation/comparison.json"
  if [[ "${OPENLORIS_RUN_SCAN_OVERLAP_ABLATIONS:-false}" == "true" ]]; then
    OPENLORIS_SEQUENCE="$sequence" bash tools/evaluation/run_gtsam_scan_overlap_ablation.sh
  fi
  if [[ ! -s "$report" ]]; then
    echo "Missing fixed-graph ablation for $sequence: $report" >&2
    echo "Run OPENLORIS_SEQUENCE=$sequence bash scripts/acceptance_test.sh openloris-scan-overlap-ablation" >&2
    exit 2
  fi
  REPORT_ARGS+=(--report "$sequence=$report")
done

# 聚合层不做宏平均粉饰：任一序列 ATE/P95 退化都会阻止默认启用门控。
python3 tools/evaluation/compare_gtsam_scan_overlap_sequences.py \
  "${REPORT_ARGS[@]}" \
  --output "$WORKSPACE/docs/evidence/gtsam_scan_overlap_multisequence.json" \
  --markdown "$WORKSPACE/docs/evidence/gtsam_scan_overlap_multisequence.md"
