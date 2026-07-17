#!/usr/bin/env bash
# Registered evaluation acceptance handlers; sourced by the Python runner.

accept_lora_q8_pipeline() {
  bash scripts/build_qwen_lora_q8.sh --dry-run
}

accept_lora_q8_comparison() {
  bash tools/evaluation/evaluate_lora_q8_comparison.sh
  python3 tools/evaluation/audit_lora_q8_pipeline.py --strict-reproduced
}

accept_llama_decode_benchmark() {
  check_llama_cpp_runtime
  require_file third_party/llama.cpp/build/bin/llama-bench
  python3 tools/evaluation/benchmark_llama_decode_speed.py \
    --minimum-decode-tokens-per-s "${LLAMA_DECODE_MIN_TOKENS_PER_S:-8.6}" \
    ${LLAMA_BENCH_NO_WARMUP:+--no-warmup}
}

accept_slam_benchmark() {
  bash scripts/smoke_test_slam_mapping_baseline.sh
}

accept_slam_gtsam_benchmark() {
  SLAM_SOLVER=gtsam bash scripts/smoke_test_slam_mapping_baseline.sh
}

accept_slam_ab_benchmark() {
  SLAM_SOLVER=ceres bash scripts/smoke_test_slam_mapping_baseline.sh
  SLAM_SOLVER=gtsam bash scripts/smoke_test_slam_mapping_baseline.sh
  python3 tools/evaluation/compare_slam_backends.py
}

accept_slam_evaluation_stage() {
  run_slam_evaluation_stage
}

accept_openloris_groundtruth() {
  python3 tools/evaluation/setup_openloris_groundtruth.py \
    --sequence "${OPENLORIS_SEQUENCE:-office1-1}" \
    --output-root "${OPENLORIS_ROOT:-datasets/openloris}"
}

accept_openloris_rosbag_setup() {
  ROSBAG_ARGS=(
    --sequence "${OPENLORIS_SEQUENCE:-office1-1}"
    --output-root "${OPENLORIS_ROOT:-datasets/openloris}"
  )
  if [[ -n "${OPENLORIS_ARCHIVE:-}" ]]; then
    ROSBAG_ARGS+=(--archive "$OPENLORIS_ARCHIVE")
  fi
  if [[ "${OPENLORIS_NO_DOWNLOAD:-false}" == "true" ]]; then
    ROSBAG_ARGS+=(--no-download)
  fi
  if [[ "${OPENLORIS_RANGE_ONLY:-false}" == "true" ]]; then
    ROSBAG_ARGS+=(--range-only)
  fi
  if [[ "${OPENLORIS_DIRECT_BAG:-false}" == "true" ]]; then
    ROSBAG_ARGS+=(--direct-bag)
  fi
  ROSBAG_ARGS+=(--download-connections "${OPENLORIS_DOWNLOAD_CONNECTIONS:-8}")
  python3 tools/evaluation/setup_openloris_rosbag.py "${ROSBAG_ARGS[@]}"
}

accept_openloris_sequence_ranking() {
  OPENLORIS_ROOT="${OPENLORIS_ROOT:-$WORKSPACE/datasets/openloris}"
  python3 tools/evaluation/setup_openloris_groundtruth.py \
    --sequence "${OPENLORIS_SEQUENCE:-office1-1}" --output-root "$OPENLORIS_ROOT"
  python3 tools/evaluation/rank_openloris_revisit_sequences.py \
    --archive "$OPENLORIS_ROOT/archive/groundtruth.zip" \
    --sensor-contracts "$WORKSPACE/src/embodied_slam/config/openloris_sensor_contracts.json" \
    --output "${OPENLORIS_RANKING_REPORT:-logs/openloris/revisit_sequence_ranking.json}"
}

accept_openloris_long_loop_evidence() {
  bash tools/evaluation/run_openloris_long_loop_evidence.sh
}

accept_openloris_robust_kernel_ablation() {
  bash tools/evaluation/run_gtsam_robust_kernel_ablation.sh
}

accept_openloris_loop_consistency_ablation() {
  local sequence="${OPENLORIS_SEQUENCE:-corridor1-1}"
  local source_dir="${OPENLORIS_OUTPUT_DIR:-$WORKSPACE/logs/openloris/$sequence}"
  GTSAM_INCLUDE_CONSISTENCY_GATE=true \
  GTSAM_ABLATION_OUTPUT_DIR="${GTSAM_CONSISTENCY_OUTPUT_DIR:-$source_dir/loop_consistency_ablation}" \
    bash tools/evaluation/run_gtsam_robust_kernel_ablation.sh
}

accept_openloris_scan_overlap_ablation() {
  bash tools/evaluation/run_gtsam_scan_overlap_ablation.sh
}

accept_openloris_scan_overlap_multisequence() {
  bash tools/evaluation/run_gtsam_scan_overlap_multisequence.sh
}

accept_openloris_lidar_loop_candidates() {
  bash tools/evaluation/run_openloris_lidar_loop_candidates.sh
}

accept_openloris_lidar_shadow_matches() {
  bash tools/evaluation/run_openloris_lidar_shadow_matches.sh
}

accept_openloris_lidar_submap_ablation() {
  bash tools/evaluation/run_openloris_lidar_submap_ablation.sh
}

accept_openloris_replay_stage() {
  python3 -c 'import rosbags' || {
    echo "Missing optional replay runtime; run: pip install -r requirements-slam-eval.txt" >&2
    exit 2
  }
  colcon build --packages-up-to embodied_slam embodied_slam_tools --symlink-install
  colcon test --packages-select embodied_slam embodied_slam_tools --event-handlers console_direct+
  colcon test-result --test-result-base build/embodied_slam --verbose
  colcon test-result --test-result-base build/embodied_slam_tools --verbose
  python3 -m pytest -q \
    tests/evaluation/test_openloris_backend_comparison.py \
    tests/evaluation/test_openloris_rosbag_setup.py \
    tests/evaluation/test_openloris_experiment_manifest.py \
    tests/evaluation/test_slam_degradation_analysis.py
  bash scripts/smoke_test_openloris_replay_adapter.sh
}

accept_openloris_bag_preflight() {
  OPENLORIS_SEQUENCE="${OPENLORIS_SEQUENCE:-office1-1}"
  OPENLORIS_ROOT="${OPENLORIS_ROOT:-$WORKSPACE/datasets/openloris}"
  OPENLORIS_BAG="${OPENLORIS_BAG:-$OPENLORIS_ROOT/rosbag/$OPENLORIS_SEQUENCE/$OPENLORIS_SEQUENCE.bag}"
  if [[ ! -f "$OPENLORIS_BAG" ]]; then
    echo "Missing $OPENLORIS_BAG; run openloris-rosbag-setup first" >&2
    exit 2
  fi
  python3 -c 'import rosbags' || {
    echo "Missing rosbags; run: pip install -r requirements-slam-eval.txt" >&2
    exit 2
  }
  ros2 run embodied_slam_tools openloris_rosbag_inspect "$OPENLORIS_BAG" \
    --output "${OPENLORIS_CONTRACT_REPORT:-logs/openloris_bag_contract.json}"
}

accept_openloris_slam_ceres() {
  bash tools/evaluation/run_openloris_slam_replay.sh ceres
}

accept_openloris_slam_gtsam() {
  bash tools/evaluation/run_openloris_slam_replay.sh gtsam
}

accept_openloris_slam_ab() {
  bash tools/evaluation/run_openloris_slam_replay.sh ceres
  bash tools/evaluation/run_openloris_slam_replay.sh gtsam
  OPENLORIS_SEQUENCE="${OPENLORIS_SEQUENCE:-office1-1}"
  OPENLORIS_OUTPUT_DIR="${OPENLORIS_OUTPUT_DIR:-$WORKSPACE/logs/openloris/$OPENLORIS_SEQUENCE}"
  python3 tools/evaluation/compare_openloris_backends.py \
    --ceres "$OPENLORIS_OUTPUT_DIR/ceres_report.json" \
    --gtsam "$OPENLORIS_OUTPUT_DIR/gtsam_report.json" \
    --ceres-degradation "$OPENLORIS_OUTPUT_DIR/ceres_degradation.json" \
    --gtsam-degradation "$OPENLORIS_OUTPUT_DIR/gtsam_degradation.json" \
    --output "$OPENLORIS_OUTPUT_DIR/backend_comparison.json"
}

accept_openloris_loop_evidence() {
  OPENLORIS_SEQUENCE="${OPENLORIS_SEQUENCE:-office1-7}"
  OPENLORIS_ROOT="${OPENLORIS_ROOT:-$WORKSPACE/datasets/openloris}"
  OPENLORIS_REFERENCE="$OPENLORIS_ROOT/groundtruth/$OPENLORIS_SEQUENCE/groundtruth.txt"
  if [[ ! -s "$OPENLORIS_REFERENCE" ]]; then
    python3 tools/evaluation/setup_openloris_groundtruth.py \
      --sequence "$OPENLORIS_SEQUENCE" --output-root "$OPENLORIS_ROOT"
  fi
  python3 tools/evaluation/analyze_openloris_revisits.py \
    --reference "$OPENLORIS_REFERENCE" \
    --output "${OPENLORIS_REVISIT_REPORT:-logs/openloris/$OPENLORIS_SEQUENCE/revisit_catalog.json}" \
    --loop-radius "${SLAM_LOOP_RADIUS_M:-0.50}" \
    --loop-yaw-tolerance-deg "${SLAM_LOOP_YAW_TOLERANCE_DEG:-30.0}" \
    --min-events "${OPENLORIS_MIN_REVISIT_EVENTS:-1}"
  colcon build --packages-up-to embodied_slam embodied_slam_tools --symlink-install
  OPENLORIS_BAG="${OPENLORIS_BAG:-$OPENLORIS_ROOT/rosbag/$OPENLORIS_SEQUENCE/$OPENLORIS_SEQUENCE.bag}"
  DEFAULT_OPENLORIS_ANNOTATIONS=""
  if [[ "$OPENLORIS_SEQUENCE" == "office1-7" ]]; then
    DEFAULT_OPENLORIS_ANNOTATIONS="$WORKSPACE/src/embodied_slam/config/openloris_office1_7_annotations.json"
  fi
  OPENLORIS_ANNOTATIONS="${OPENLORIS_ANNOTATIONS:-$DEFAULT_OPENLORIS_ANNOTATIONS}"
  if [[ ! -s "$OPENLORIS_BAG" ]]; then
    if [[ "$OPENLORIS_SEQUENCE" == "market1-3" ]]; then
      OPENLORIS_SEQUENCE="$OPENLORIS_SEQUENCE" OPENLORIS_ROOT="$OPENLORIS_ROOT" \
        OPENLORIS_DIRECT_BAG=true bash scripts/acceptance_test.sh openloris-rosbag-setup
    else
      OPENLORIS_SEQUENCE="$OPENLORIS_SEQUENCE" OPENLORIS_ROOT="$OPENLORIS_ROOT" \
        OPENLORIS_RANGE_ONLY=true bash scripts/acceptance_test.sh openloris-rosbag-setup
    fi
  fi
  OPENLORIS_SEQUENCE="$OPENLORIS_SEQUENCE" OPENLORIS_ROOT="$OPENLORIS_ROOT" \
    OPENLORIS_BAG="$OPENLORIS_BAG" OPENLORIS_EVALUATE_LOOP_CONSTRAINTS=true \
    OPENLORIS_EVALUATE_FRONTEND=true \
    OPENLORIS_ANNOTATIONS="$OPENLORIS_ANNOTATIONS" \
    bash tools/evaluation/run_openloris_slam_replay.sh gtsam
}

accept_openloris_loop_sweep() {
  colcon build --packages-up-to embodied_slam embodied_slam_tools --symlink-install
  bash tools/evaluation/run_openloris_loop_sweep.sh
}

accept_openloris_evaluate() {
  if [[ -z "${SLAM_ESTIMATE_FILE:-}" ]]; then
    echo "Usage: SLAM_ESTIMATE_FILE=/path/to/estimate.tum ${ACCEPTANCE_PROGRAM:-acceptance_test.sh} openloris-evaluate" >&2
    exit 2
  fi
  OPENLORIS_SEQUENCE="${OPENLORIS_SEQUENCE:-office1-1}"
  OPENLORIS_ROOT="${OPENLORIS_ROOT:-datasets/openloris}"
  OPENLORIS_REFERENCE="$OPENLORIS_ROOT/groundtruth/$OPENLORIS_SEQUENCE/groundtruth.txt"
  if [[ ! -s "$OPENLORIS_REFERENCE" ]]; then
    python3 tools/evaluation/setup_openloris_groundtruth.py \
      --sequence "$OPENLORIS_SEQUENCE" --output-root "$OPENLORIS_ROOT"
  fi
  python3 tools/evaluation/evaluate_slam_trajectory.py \
    --reference "$OPENLORIS_REFERENCE" \
    --estimate "$SLAM_ESTIMATE_FILE" \
    --output "${SLAM_EVALUATION_REPORT:-logs/openloris_${OPENLORIS_SEQUENCE}_report.json}" \
    --max-time-diff "${SLAM_MAX_TIME_DIFF_S:-0.05}" \
    --rpe-delta "${SLAM_RPE_DELTA_S:-1.0}" \
    --min-match-ratio "${SLAM_MIN_MATCH_RATIO:-0.80}" \
    ${SLAM_MAX_ATE_RMSE_M:+--max-ate-rmse "$SLAM_MAX_ATE_RMSE_M"} \
    ${SLAM_MAX_RPE_RMSE_M:+--max-rpe-translation-rmse "$SLAM_MAX_RPE_RMSE_M"}
}

accept_dynamic_obstacle_ablation() {
  colcon build --packages-up-to embodied_navigation --symlink-install --allow-overriding \
    embodied_agent_interfaces embodied_navigation
  set +u
  source install/setup.bash
  set -u
  run_dynamic_obstacle_ablation
}

accept_dynamic_obstacle_navigation_ablation() {
  bash scripts/run_dynamic_obstacle_navigation_ablation.sh
}

accept_instruction_eval_dataset() {
  python3 tools/evaluation/validate_instruction_eval_dataset.py
}

accept_instruction_parser_eval() {
  python3 tools/evaluation/evaluate_instruction_parser.py --minimum "${INSTRUCTION_PARSER_MINIMUM:-1.0}"
}

accept_instruction_following_eval() {
  check_llama_cpp_runtime
  bash tools/evaluation/evaluate_instruction_following.sh \
    --minimum "${INSTRUCTION_FOLLOWING_MINIMUM:-0.0}" \
    --minimum-effective "${INSTRUCTION_FOLLOWING_EFFECTIVE_MINIMUM:-0.0}"
}

accept_instruction_following_lora_candidates() {
  if [[ ! -s "${INSTRUCTION_FOLLOWING_REPORT:-logs/instruction_following_report.json}" ]]; then
    echo "Missing instruction following report; run: bash scripts/acceptance_test.sh instruction-following-eval" >&2
    exit 1
  fi
  python3 tools/evaluation/export_instruction_following_lora_candidates.py \
    --report "${INSTRUCTION_FOLLOWING_REPORT:-logs/instruction_following_report.json}" \
    --dataset "${INSTRUCTION_FOLLOWING_DATASET:-training/robot_dialogue_seed.jsonl}" \
    --output "${INSTRUCTION_FOLLOWING_LORA_CANDIDATES:-training/robot_dialogue_lora_candidates.jsonl}" \
    --metadata-output "${INSTRUCTION_FOLLOWING_LORA_CANDIDATES_META:-training/robot_dialogue_lora_candidates.meta.json}" \
    ${INSTRUCTION_FOLLOWING_LORA_FAIL_IF_EMPTY:+--fail-if-empty}
}

accept_instruction_following_lora_review() {
  LORA_REVIEW_ARGS=(
    --candidates "${INSTRUCTION_FOLLOWING_LORA_CANDIDATES:-training/robot_dialogue_lora_candidates.jsonl}"
    --review "${INSTRUCTION_FOLLOWING_LORA_REVIEW:-training/robot_dialogue_lora_review.json}"
    --output "${INSTRUCTION_FOLLOWING_LORA_APPROVED:-training/robot_dialogue_lora_approved.jsonl}"
    --metadata-output "${INSTRUCTION_FOLLOWING_LORA_APPROVED_META:-training/robot_dialogue_lora_approved.meta.json}"
  )
  if [[ "${INSTRUCTION_FOLLOWING_LORA_REVIEW_INIT:-false}" == "true" ]]; then
    LORA_REVIEW_ARGS+=(--init-review)
  fi
  if [[ -n "${INSTRUCTION_FOLLOWING_LORA_FAIL_IF_EMPTY:-}" ]]; then
    LORA_REVIEW_ARGS+=(--fail-if-empty)
  fi
  python3 tools/evaluation/review_lora_candidates.py "${LORA_REVIEW_ARGS[@]}"
}

accept_continuous_voice_benchmark() {
  CHECK_MODE="${1:-offline}"
  if [[ "$CHECK_MODE" != "offline" && "$CHECK_MODE" != "online" ]]; then
    echo "Usage: ${ACCEPTANCE_PROGRAM:-acceptance_test.sh} continuous-voice-benchmark {offline|online}" >&2
    exit 2
  fi
  REPORT_PATH="${VOICE_BENCHMARK_LIVE_REPORT:-logs/continuous_voice_${CHECK_MODE}_live_report.json}"
  SUMMARY_PATH="${VOICE_BENCHMARK_REPORT:-logs/voice_benchmark_${CHECK_MODE}_report.json}"
  REPORT_PATH="$(realpath -m "$REPORT_PATH")"
  SUMMARY_PATH="$(realpath -m "$SUMMARY_PATH")"
  CONTROL_ARGS=()
  if [[ "${VOICE_BENCHMARK_CONTROL_MANAGED:-false}" == "true" ]]; then
    CONTROL_ARGS+=(--control-managed)
    echo "连续语音控制链路由一键留证脚本后台管理。"
  else
    echo "请先在另一个终端启动 acceptance_test.sh continuous-$CHECK_MODE"
  fi
  LIVE_STATUS=0
  if python3 scripts/continuous_live_check.py \
      --scenario benchmark \
      --capture-source real_microphone \
      --agent-mode "$CHECK_MODE" \
      "${CONTROL_ARGS[@]}" \
      --duration "${VOICE_BENCHMARK_DURATION:-300}" \
      --progress-interval "${VOICE_BENCHMARK_PROGRESS_INTERVAL:-15}" \
      --min-asr 12 \
      --min-candidates 10 \
      --min-success 9 \
      --output "$REPORT_PATH"; then
    LIVE_STATUS=0
  else
    LIVE_STATUS=$?
  fi
  SUMMARY_STATUS=0
  if python3 tools/evaluation/evaluate_live_voice_benchmark.py \
      --report "$REPORT_PATH" \
      --output "$SUMMARY_PATH"; then
    SUMMARY_STATUS=0
  else
    SUMMARY_STATUS=$?
  fi
  echo "现场事件报告：$REPORT_PATH"
  echo "量化汇总报告：$SUMMARY_PATH"
  if (( LIVE_STATUS != 0 || SUMMARY_STATUS != 0 )); then
    echo "FAIL: 报告已经生成，但当前指标未达到门槛；请把上述两个文件发给 Codex。" >&2
    exit 1
  fi
  echo "PASS: 五分钟连续语音量化验收通过"
}

accept_voice_benchmark_report() {
  REPORT_PATH="${1:-}"
  if [[ -z "$REPORT_PATH" ]]; then
    echo "Usage: ${ACCEPTANCE_PROGRAM:-acceptance_test.sh} voice-benchmark-report REPORT_FILE" >&2
    exit 2
  fi
  python3 tools/evaluation/evaluate_live_voice_benchmark.py --report "$REPORT_PATH"
}
