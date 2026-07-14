#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
cd "$WORKSPACE"

reports=()
for model in current_only constant_velocity kalman imm; do
  report="$WORKSPACE/logs/dynamic_obstacle_navigation_${model}.json"
  echo "[dynamic navigation ablation] model=$model"
  DYNAMIC_MOTION_MODEL="$model" \
    DYNAMIC_NAVIGATION_REPORT="$report" \
    bash scripts/smoke_test_predicted_dynamic_obstacle_navigation.sh
  reports+=("$report")
done

python3 scripts/compare_dynamic_navigation_models.py \
  "${reports[@]}" --output logs/dynamic_obstacle_navigation_ablation.json
