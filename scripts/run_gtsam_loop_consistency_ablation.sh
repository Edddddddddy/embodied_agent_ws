#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
SEQUENCE="${OPENLORIS_SEQUENCE:-corridor1-1}"
SOURCE_DIR="${OPENLORIS_OUTPUT_DIR:-$WORKSPACE/logs/openloris/$SEQUENCE}"

GTSAM_INCLUDE_CONSISTENCY_GATE=true \
GTSAM_ABLATION_OUTPUT_DIR="${GTSAM_CONSISTENCY_OUTPUT_DIR:-$SOURCE_DIR/loop_consistency_ablation}" \
bash "$WORKSPACE/scripts/run_gtsam_robust_kernel_ablation.sh"
