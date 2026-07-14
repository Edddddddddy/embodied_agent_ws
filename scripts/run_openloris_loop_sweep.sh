#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
cd "$WORKSPACE"

SEQUENCE="${OPENLORIS_SEQUENCE:-office1-7}"
ROOT="${OPENLORIS_ROOT:-$WORKSPACE/datasets/openloris}"
RAW_BAG="${OPENLORIS_RAW_BAG:-$ROOT/rosbag/$SEQUENCE/$SEQUENCE.bag}"
RAW_SOURCE="${OPENLORIS_RAW_SOURCE:-$(dirname "$RAW_BAG")/source.json}"
DERIVED_DIR="${OPENLORIS_DERIVED_DIR:-$ROOT/derived/$SEQUENCE}"
DERIVED_BAG="${OPENLORIS_DERIVED_BAG:-$DERIVED_DIR/${SEQUENCE}-slam.bag}"
DERIVED_SOURCE="${OPENLORIS_DERIVED_SOURCE:-$DERIVED_DIR/source.json}"
RESULT_ROOT="${OPENLORIS_SWEEP_OUTPUT_DIR:-$WORKSPACE/logs/openloris/$SEQUENCE/loop_sweep}"
CONFIG_DIR="$RESULT_ROOT/configs"
CONFIG_MANIFEST="$RESULT_ROOT/config_manifest.json"
MATRIX="${OPENLORIS_SWEEP_MATRIX:-$WORKSPACE/src/embodied_slam/config/openloris_loop_sweep.json}"
BASE_CONFIG="${OPENLORIS_SWEEP_BASE_CONFIG:-$WORKSPACE/src/embodied_slam/config/openloris_mapping_gtsam.yaml}"
ANNOTATIONS="${OPENLORIS_ANNOTATIONS:-$WORKSPACE/src/embodied_slam/config/openloris_office1_7_annotations.json}"

if [[ ! -s "$RAW_BAG" || ! -s "$RAW_SOURCE" ]]; then
  echo "Missing verified raw OpenLORIS bag/source for $SEQUENCE" >&2
  echo "Run: OPENLORIS_SEQUENCE=$SEQUENCE OPENLORIS_RANGE_ONLY=true bash scripts/acceptance_test.sh openloris-rosbag-setup" >&2
  exit 2
fi

if [[ -e "$DERIVED_BAG" || -e "$DERIVED_SOURCE" ]]; then
  if [[ ! -s "$DERIVED_BAG" || ! -s "$DERIVED_SOURCE" ]]; then
    echo "Derived bag/source is incomplete; inspect $DERIVED_DIR before retrying" >&2
    exit 2
  fi
else
  VERIFY_ARGS=()
  if [[ "${OPENLORIS_VERIFY_RAW_HASH:-false}" == "true" ]]; then
    VERIFY_ARGS+=(--verify-source-hash)
  fi
  python3 scripts/compact_openloris_rosbag.py \
    --input "$RAW_BAG" --source "$RAW_SOURCE" \
    --output "$DERIVED_BAG" --output-source "$DERIVED_SOURCE" \
    --workspace "$WORKSPACE" "${VERIFY_ARGS[@]}"
fi

mkdir -p "$RESULT_ROOT"
python3 scripts/build_openloris_loop_sweep_configs.py \
  --base "$BASE_CONFIG" --matrix "$MATRIX" \
  --output-dir "$CONFIG_DIR" --manifest "$CONFIG_MANIFEST"

mapfile -t PROFILES < <(
  python3 -c 'import json,sys; print("\n".join(p["id"] for p in json.load(open(sys.argv[1]))["profiles"]))' \
    "$CONFIG_MANIFEST"
)
for profile in "${PROFILES[@]}"; do
  echo "[loop-sweep] profile=$profile"
  PROFILE_MANIFEST="$RESULT_ROOT/$profile/gtsam_manifest.json"
  if [[ "${OPENLORIS_SWEEP_RESUME:-true}" == "true" && -s "$PROFILE_MANIFEST" ]]; then
    if python3 - "$PROFILE_MANIFEST" "$CONFIG_DIR/$profile.yaml" "$DERIVED_SOURCE" <<'PY'
import hashlib
import json
import sys
from pathlib import Path

manifest = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
config_hash = hashlib.sha256(Path(sys.argv[2]).read_bytes()).hexdigest()
source = json.loads(Path(sys.argv[3]).read_text(encoding="utf-8"))
valid = (
    manifest.get("passed") is True
    and int(manifest.get("schema_version", 0)) >= 2
    and manifest.get("checks", {}).get("derived_topic_subset_bound") is True
    and manifest["configuration"]["params"]["sha256"] == config_hash
    and manifest["dataset"]["bag_sha256"] == source["bag_sha256"]
)
raise SystemExit(0 if valid else 1)
PY
    then
      echo "[loop-sweep] reuse verified result: $PROFILE_MANIFEST"
      continue
    fi
  fi
  OPENLORIS_SEQUENCE="$SEQUENCE" \
  OPENLORIS_ROOT="$ROOT" \
  OPENLORIS_BAG="$DERIVED_BAG" \
  OPENLORIS_BAG_SOURCE="$DERIVED_SOURCE" \
  OPENLORIS_PARAMS_FILE="$CONFIG_DIR/$profile.yaml" \
  OPENLORIS_OUTPUT_DIR="$RESULT_ROOT/$profile" \
  OPENLORIS_REPLAY_RATE="${OPENLORIS_REPLAY_RATE:-1.0}" \
  OPENLORIS_STARTUP_DELAY_S="${OPENLORIS_STARTUP_DELAY_S:-3.0}" \
  OPENLORIS_EVALUATE_LOOP_CONSTRAINTS=true \
  OPENLORIS_ANNOTATIONS="$ANNOTATIONS" \
    bash scripts/run_openloris_slam_replay.sh gtsam
done

python3 scripts/compare_openloris_loop_sweep.py \
  --configs "$CONFIG_MANIFEST" --result-root "$RESULT_ROOT" \
  --output "$RESULT_ROOT/comparison.json"
echo "PASS: real OpenLORIS accepted-edge threshold sweep"
echo "Evidence: $RESULT_ROOT/comparison.json"
