#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# 统一由脚本位置推导仓库根目录，避免 feature worktree 静默加载主工作区 install。
source "$SCRIPT_DIR/lifecycle_utils.sh"
embodied_resolve_workspace "${BASH_SOURCE[0]}"
LEVEL="${1:-mock}"

usage() {
  cat <<'EOF'
Usage: acceptance_test.sh MODE

Recommended public modes:
  core                       Fast repository, Python and C++ developer gate
  robotics-gate              Unified ROS/Nav2/SLAM/dynamic-obstacle evidence gate
  continuous-multi-command   One utterance -> ordered command queue regression
  continuous-offline         Live microphone -> offline Agent -> simulation
  continuous-online          Live microphone -> online Agent -> simulation
  gazebo                     Typed ROS 2 Action -> Gazebo physical-motion check
  nav2-stage                 Lightweight voice navigation/patrol stage gate
  slam-nav-showcase-stage    Realistic scene/NLU/stage-orchestration gate
  voice-slam-workplace-demo  Live voice workplace mapping and navigation demo
  slam-evaluation-stage      Deterministic ATE/RPE/loop-correction evaluation
  openloris-replay-stage     Public-bag Ceres/GTSAM replay evidence
  dynamic-obstacle-stage     Tracker/predictor/costmap-plugin stage gate
  dynamic-obstacle-navigation Full predicted-obstacle Nav2 replan demonstration

Run `bash scripts/acceptance_test.sh --help-all` for advanced/internal modes.
EOF
}

usage_all() {
  cat <<'EOF'
Usage: acceptance_test.sh MODE

Automated modes:
  core                Typical developer gate: repository, Python unit, C++ unit tests
  architecture-facts  Verify generated package/CI/CLI/release-gate architecture evidence
  agent-lifecycle     Online/offline configure -> activate -> deactivate -> reactivate
  preflight           Check offline model/runtime files
  mock                Build, unit tests, and dependency-free ROS smokes
  online              Minimal-token live ASR/LLM/TTS verification
  offline             Real ZipFormer/llama.cpp/Sherpa-TTS verification
  llama-cpp-preflight Check llama.cpp binary/model plus llama-server health/models API
  llama-cpp-smoke     Low-token llama.cpp streaming chat verification
  llama-decode-benchmark Measure llama.cpp CPU decode tokens/s with llama-bench
  offline-runtime-versions Check pinned llama.cpp/SummerTTS/sherpa-onnx versions
  offline-showcase-report Generate offline deployment JSON/Markdown evidence report
  offline-evidence-audit Audit offline report evidence and over-claiming boundaries
  offline-latency     Check llama.cpp first token and Sherpa short-sentence synthesis latency
  offline-voice-e2e-report Real ZipFormer -> llama.cpp -> pseudo-streaming TTS latency evidence
  summer-tts-preflight Check SummerTTS source/binary/model runtime files
  summer-tts-smoke     Real SummerTTS synthesis verification
  summer-pseudo-tts    Real SummerTTS + pseudo-streaming double-buffer verification
  summer-tts-service   Resident C++ ROS SummerTTS service smoke
  summer-tts-cache-audit Audit SummerTTS short-feedback cache latency evidence
  pseudo-tts          Dependency-free llama-style stream + pseudo TTS pipeline smoke
  sherpa-asr-preflight ASR-only check: sherpa_onnx import + ZipFormer model files
  sherpa-asr-smoke    ASR-only real decode on bundled ZipFormer test wav
  offline-sherpa-typed Real Sherpa ASR/TTS + llama.cpp through typed Action simulation
  demo                Rich mock demo: ordered actions, accessories, and arc motion
  navigation-demo     Voice-style target navigation and multi-waypoint patrol smoke
  nav2-bridge         Voice navigation commands are converted to Nav2 action goals
  nav2-preflight      Check Nav2/TurtleBot3 voice launch dependencies and arguments
  nav2-assets         Audit Nav2 voice demo places/RViz/launch assets
  nav2-stage          Stage gate for voice navigation/patrol; excludes heavy Gazebo/Nav2
  nav2-turtlebot3     Heavy Gazebo/Nav2 run: voice text drives target navigation/patrol
  nav2-resilience     Heavy Gazebo/Nav2 run: dynamic replan + unreachable failure
  mapping-stage       Build/test/audit the controlled-drift SLAM mapping baseline
  slam-nav-showcase-stage Audit realistic scene, semantic goals, and mapping/navigation stages
  slam-nav-showcase   Heavy realistic apartment + AMCL + Nav2 motion gate
  slam-nav-showcase-mapping Heavy realistic apartment + SLAM map-save gate
  slam-session-orchestrator-stage Typed single-terminal mapping/save/navigation FSM gate
  slam-autonomous-mission-stage One intent -> exploration/save/localization/navigation FSM gate
  slam-autonomous-mission Heavy one-intent frontier SLAM -> AMCL/Nav2 mission
  slam-session-orchestrator Heavy one-terminal Gazebo mapping/save/restart/navigation gate
  slam-benchmark      Heavy Gazebo run: fixed loop, 5 cm map, and drift metrics report
  slam-gtsam-benchmark Heavy Gazebo run with the project GTSAM ScanSolver plugin
  slam-ab-benchmark   Run Ceres/GTSAM on the same scenario and compare evidence
  slam-navigation     Heavy run: saved map -> AMCL -> Nav2 plan -> goal execution
  slam-evaluation-stage Synthetic ATE/RPE/loop-correction gate without Gazebo or downloads
  openloris-groundtruth Download and verify one public OpenLORIS ground-truth trajectory
  openloris-rosbag-setup Resume and verify a tar-range or standalone public rosbag
  openloris-sequence-ranking Rank all ground-truth trajectories before large bag download
  openloris-long-loop-evidence Verified corridor1-1 long-loop GTSAM/frontend experiment
  openloris-robust-kernel-ablation Re-optimize one fixed graph with none/Huber/Cauchy
  openloris-loop-consistency-ablation Reject geometrically inconsistent non-local graph edges
  openloris-scan-overlap-ablation Validate accepted constraints with no-GT scan overlap evidence
  openloris-scan-overlap-multisequence Aggregate independent fixed-graph overlap ablations
  openloris-lidar-loop-candidates Evaluate C++ LiDAR loop retrieval on two real sequences
  openloris-lidar-shadow-matches Evaluate C++ shadow scan matching on two real sequences
  openloris-lidar-submap-ablation Compare scan-to-scan and local-submap shadow matching
  lidar-loop-runtime  Lifecycle scan -> submap verification -> shadow gate -> guarded Karto adapter
  openloris-replay-stage Generate a tiny bag and replay it through Ceres/GTSAM SLAM
  openloris-bag-preflight Validate OPENLORIS_BAG topics, frames, and optional runtime
  openloris-slam-ceres Replay a real OpenLORIS bag through the Ceres backend
  openloris-slam-gtsam Replay a real OpenLORIS bag through the GTSAM backend
  openloris-slam-ab    Run both backends on one bag and compare their reports
  openloris-loop-evidence Audit office1-7 revisits and run accepted-loop evaluation
  openloris-loop-sweep Build a lossless topic subset and sweep loop-front-end thresholds
  openloris-evaluate  Evaluate SLAM_ESTIMATE_FILE against an OpenLORIS sequence
  dynamic-obstacle-stage Build/test tracker, motion predictor, and Nav2 costmap plugin seam
  dynamic-obstacle-ablation Compare current-only/CV/Kalman/IMM on one fixed C++ scenario
  dynamic-obstacle-navigation Heavy run: predicted crossing obstacle -> Nav2 replan -> goal
  dynamic-obstacle-navigation-ablation Heavy four-model Gazebo/Nav2 crossing comparison
  continuous-mock     One wake word, several queued commands, and sleep gate
  continuous-soak     Long wake session keeps accepting many queued commands
  continuous-endpoint Endpoint speech_ended commits feed continuous ASR commands
  continuous-multi-command NLU parses one ASR final into ordered queued commands
  continuous-navigation Multi-target navigation and patrol commands queue in a continuous session
  continuous-navigation-natural Natural multi-target speech becomes waypoint patrol commands
  continuous-queue-full Busy continuous queue rejects excess commands with feedback
  continuous-ttl      Busy continuous queue expires stale non-priority commands
  continuous-timeout  Voice session timeout requires a fresh wake word
  continuous-kws-mock KWS sidecar opens a continuous session and executes a command
  speaker-memory-mock Speaker identity and per-user memory smoke test
  speaker-enroll     Speaker enrollment request saves wav samples and speakers.txt
  speaker-runtime    Real sherpa speaker embedding self-match and ambiguity guard
  vad-sidecar         Dependency-free Silero VAD sidecar seam smoke test
  silero-vad-runtime  Real lightweight Silero ONNX inference and latency report
  webrtc-vad-sidecar  Installed WebRTC VAD sidecar runtime smoke test
  kws-sidecar         Dependency-free keyword wake sidecar seam smoke test
  sherpa-kws-sidecar  Installed Sherpa-ONNX KWS runtime startup smoke test
  openwakeword-sidecar Dependency-free openWakeWord adapter runtime smoke test
  livekit-sidecar     Dependency-free LiveKit WakeWord adapter runtime smoke test
  kws-calibration     Dependency-free KWS score calibration smoke test
  voice-readiness     Dependency-free voice readiness smoke test
  provider-preflight  Optional VAD/KWS provider unit tests plus current-env preflight
  voice-stability-preflight Strict preflight requiring Silero/WebRTC mature VAD
  voice-vad-runtime-dry-run Show optional WebRTC/Silero VAD install commands without installing
  voice-kws-runtime-dry-run Show optional openWakeWord/sherpa KWS install commands without installing
  voice-calibration-report Generate voice profile/threshold calibration report
  instruction-eval-dataset Validate lightweight robot instruction eval dataset
  instruction-parser-eval Evaluate deterministic command parser on instruction eval set
  instruction-following-eval Evaluate offline LLM instruction following with llama.cpp
  instruction-following-lora-candidates Export failed instruction-following cases for LoRA review
  instruction-following-lora-review Init/apply human review for approved LoRA dataset
  lora-q8-pipeline    Dry-run LoRA merge -> GGUF -> Q8 pipeline and artifact audit
  lora-q8-comparison  Run isolated baseline/tuned Q8 holdout evaluation and audit
  asr-nlu-samples-to-eval Convert live ASR/NLU sample JSONL into reviewable eval candidates
  asr-nlu-candidate-eval Evaluate parser accuracy on reviewable ASR/NLU eval candidates
  release-gate        Job-showcase core 5-command gate with logs/acceptance_report.json
  robotics-gate       Unified ROS/Nav2/SLAM/dynamic-obstacle evidence gate
  demo-gate           Pre-demo automatic evidence gate with logs/demo_acceptance_report.json
  demo-evidence-checklist Summarize automatic/live/visual demo evidence into JSON/Markdown
  runtime-evidence-summary Summarize online/offline 5-minute, LLM, and latency evidence
  wsl-microphone-preflight PulseAudio/WSLg microphone capture check before live demos
  gazebo              Typed Action physical motion verification
  cpp-action-client   Verify C++ typed Action success/feedback/cancel/timeout lifecycle
  cpp-action-scheduler Verify C++ FIFO, priority cancel, result correlation, diagnostics
  cpp-action-bridge-lifecycle Verify inactive reject, cleanup, and reactivate
  gazebo-voice        Offline synthesized speech through typed Action to Gazebo
  gazebo-voice-online Online voice provider through typed Action to Gazebo
  all                 Run all automated release gates; excludes interactive microphone

Interactive modes:
  microphone-offline  Speak into the microphone using the offline Agent
  microphone-online   Speak into the microphone using the online Agent
  continuous-offline  Long-running microphone control using the offline Agent
  continuous-online   Long-running microphone control using the online Agent
  continuous-nav2-offline  Long-running microphone target navigation with Nav2/TurtleBot3
  continuous-nav2-online   Long-running microphone target navigation with Nav2/TurtleBot3
  continuous-nav2-evidence {offline|online}  Run Nav2 microphone demo and live-check evidence in one terminal
  continuous-voice-evidence {offline|online}  Run microphone demo and 5-minute benchmark in one terminal
  continuous-live-check {offline|online}  Observe a running live microphone demo and score evidence
  continuous-voice-benchmark {offline|online}  Score a 5-minute, 10-command microphone benchmark
  continuous-nav2-live-check {offline|online}  Score a running live Nav2 microphone demo
  continuous-live-report REPORT_FILE  Re-score a saved continuous live-check report
  voice-benchmark-report REPORT_FILE  Evaluate recognition/action/false-trigger/latency metrics
  continuous-nav2-live-report REPORT_FILE  Re-score a saved Nav2 live-check report
  voice-slam-workplace-demo {offline|online}  Live office survey -> SLAM -> Nav2
EOF
}

if [[ "$LEVEL" == "help" || "$LEVEL" == "--help" || "$LEVEL" == "-h" ]]; then
  usage
  exit 0
fi

if [[ "$LEVEL" == "--help-all" || "$LEVEL" == "help-all" ]]; then
  usage_all
  exit 0
fi

# 架构事实只读取仓库文件；把它放在 ROS 环境激活之前，保证全新 clone/worktree 也能先做契约审计。
if [[ "$LEVEL" == "architecture-facts" ]]; then
  cd "$WORKSPACE"
  python3 scripts/generate_architecture_facts.py --workspace "$WORKSPACE" --check
  exit 0
fi

source "$WORKSPACE/scripts/activate.sh"
cd "$WORKSPACE"

require_file() {
  if [[ ! -s "$1" ]]; then
    echo "MISSING: $1" >&2
    return 1
  fi
  echo "READY: $1"
}

run_dynamic_obstacle_ablation() {
  mkdir -p logs
  ros2 run embodied_navigation dynamic_obstacle_model_benchmark \
    --output logs/dynamic_obstacle_model_ablation.json
  python3 scripts/verify_dynamic_obstacle_ablation.py \
    logs/dynamic_obstacle_model_ablation.json \
    --markdown logs/dynamic_obstacle_model_ablation.md
  ros2 run embodied_navigation dynamic_obstacle_association_benchmark \
    --output logs/dynamic_obstacle_association_ablation.json
  python3 scripts/verify_dynamic_obstacle_association.py \
    logs/dynamic_obstacle_association_ablation.json \
    --markdown logs/dynamic_obstacle_association_ablation.md
  ros2 run embodied_navigation dynamic_obstacle_uncertainty_benchmark \
    --output logs/dynamic_obstacle_uncertainty_ablation.json
  python3 scripts/verify_dynamic_obstacle_uncertainty.py \
    logs/dynamic_obstacle_uncertainty_ablation.json \
    --markdown logs/dynamic_obstacle_uncertainty_ablation.md
}

run_base() {
  bash scripts/run_core_tests.sh
  colcon build --symlink-install --allow-overriding \
    embodied_agent_interfaces embodied_agent_core embodied_voice_frontend \
    embodied_agent_cpp embodied_online_agent \
    embodied_offline_agent embodied_simulation embodied_slam embodied_navigation \
    embodied_slam_tools
  colcon test --packages-select \
    embodied_agent_interfaces embodied_agent_core embodied_voice_frontend \
    embodied_agent_cpp embodied_online_agent \
    embodied_offline_agent embodied_simulation embodied_slam embodied_navigation \
    embodied_slam_tools \
    --event-handlers console_direct+
  colcon test-result --verbose
  bash scripts/smoke_test.sh
  bash scripts/smoke_test_online_wake_config.sh
  bash scripts/smoke_test_recognition_retry.sh
  bash scripts/smoke_test_audio_endpoint.sh
  bash scripts/smoke_test_silero_vad_sidecar.sh
  bash scripts/smoke_test_keyword_wake_sidecar.sh
  bash scripts/smoke_test_openwakeword_sidecar.sh
  bash scripts/smoke_test_livekit_wakeword_sidecar.sh
  bash scripts/smoke_test_kws_score_calibration.sh
  bash scripts/smoke_test_voice_readiness.sh
  python3 -m pytest -q tests/integration/test_voice_provider_preflight.py
  bash scripts/smoke_test_lifecycle.sh
  bash scripts/smoke_test_agent_lifecycle.sh online
  bash scripts/smoke_test_agent_lifecycle.sh offline
  bash scripts/smoke_test_typed_action.sh
  bash scripts/smoke_test_typed_action_server.sh
  bash scripts/smoke_test_typed_action_pipeline.sh
  bash scripts/smoke_test_cpp_action_scheduler.sh
  bash scripts/smoke_test_mock_executor.sh
  bash scripts/smoke_test_demo_sequence.sh
  bash scripts/smoke_test_navigation_sequence.sh
  bash scripts/smoke_test_continuous_voice.sh online
  bash scripts/smoke_test_continuous_voice_soak.sh online
  bash scripts/smoke_test_continuous_endpoint_asr.sh online
  bash scripts/smoke_test_continuous_navigation_queue.sh online
  bash scripts/smoke_test_continuous_queue_full.sh online
  bash scripts/smoke_test_continuous_command_ttl.sh online
  bash scripts/smoke_test_continuous_session_timeout.sh online
  bash scripts/smoke_test_continuous_kws_sidecar.sh online
  bash scripts/smoke_test_composed_executor.sh
  bash scripts/smoke_test_namespaced_executor.sh
  bash scripts/smoke_test_offline.sh
  bash scripts/smoke_test_hardware.sh
  bash scripts/smoke_test_simulation.sh
}

run_online() {
  python tests/integration/test_online_api.py
  bash scripts/smoke_test_online_real.sh
}

run_offline() {
  check_offline_runtime
  bash scripts/benchmark_offline.sh
  bash scripts/evaluate_instruction_following.sh
  bash scripts/smoke_test_offline_real.sh
  bash scripts/smoke_test_offline_voice_real.sh
  bash scripts/smoke_test_offline_sherpa_typed_simulation.sh
}

run_gazebo() {
  bash scripts/smoke_test_gazebo.sh
  bash scripts/smoke_test_gazebo_typed_action.sh
}

run_slam_evaluation_stage() {
  local fixture_dir="logs/slam_evaluation_fixture"
  python3 -m pytest -q \
    tests/repository/test_slam_trajectory_evaluation.py \
    tests/repository/test_slam_evaluation_comparison.py \
    tests/repository/test_openloris_groundtruth_setup.py \
    tests/repository/test_openloris_rosbag_setup.py \
    tests/repository/test_openloris_experiment_manifest.py \
    tests/repository/test_slam_degradation_analysis.py \
    tests/repository/test_rosbag_trajectory_adapter.py
  python3 scripts/generate_slam_evaluation_fixture.py --output-dir "$fixture_dir"
  python3 scripts/evaluate_slam_trajectory.py \
    --reference "$fixture_dir/reference.tum" \
    --estimate "$fixture_dir/dead_reckoning.tum" \
    --output "$fixture_dir/dead_reckoning_report.json"
  python3 scripts/evaluate_slam_trajectory.py \
    --reference "$fixture_dir/reference.tum" \
    --estimate "$fixture_dir/loop_corrected.tum" \
    --output "$fixture_dir/loop_corrected_report.json" \
    --max-ate-rmse 0.05 \
    --max-rpe-translation-rmse 0.05
  python3 scripts/compare_slam_evaluations.py \
    --baseline "$fixture_dir/dead_reckoning_report.json" \
    --corrected "$fixture_dir/loop_corrected_report.json" \
    --output "$fixture_dir/comparison.json"
}

check_offline_runtime() {
  require_file models/sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16/encoder-epoch-99-avg-1.int8.onnx
  require_file models/sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16/decoder-epoch-99-avg-1.int8.onnx
  require_file models/sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16/joiner-epoch-99-avg-1.int8.onnx
  require_file models/vits-melo-tts-zh_en/model.onnx
  check_llama_cpp_runtime
}

check_llama_cpp_runtime() {
  require_file models/Qwen3-0.6B-Q8_0.gguf
  require_file third_party/llama.cpp/build/bin/llama-server
}

check_summer_tts_runtime() {
  require_file third_party/SummerTTS/README.md
  require_file third_party/SummerTTS/include/SynthesizerTrn.h
  require_file third_party/SummerTTS/models/single_speaker_fast.bin
  require_file third_party/SummerTTS/build/tts_test
}

run_sherpa_asr_preflight() {
  python3 scripts/sherpa_asr_smoke.py --preflight-only
}

run_sherpa_asr_smoke() {
  python3 scripts/sherpa_asr_smoke.py --expected-substring "星期三"
}

run_isolated_ros_smoke() {
  local domain_id="$1"
  shift
  local exit_code=0
  echo "[nav2-stage] ROS_DOMAIN_ID=$domain_id $*"
  ROS_DOMAIN_ID="$domain_id" "$@" || exit_code=$?
  if (( exit_code != 0 )); then
    echo "[nav2-stage] FAILED exit=$exit_code $*" >&2
    return "$exit_code"
  fi
  # 连续启动多个 ROS graph 时，给 DDS discovery 和进程组清理留出很短缓冲。
  # 这能避免阶段门禁里相邻 smoke 互相看到上一轮残留节点。
  sleep 0.5
  echo "[nav2-stage] PASS $*"
  return 0
}

case "$LEVEL" in
  core) bash scripts/run_core_tests.sh ;;
  architecture-facts) python3 scripts/generate_architecture_facts.py --workspace "$WORKSPACE" --check ;;
  agent-lifecycle) bash scripts/smoke_test_agent_lifecycle.sh online; bash scripts/smoke_test_agent_lifecycle.sh offline ;;
  preflight) check_offline_runtime ;;
  mock) run_base ;;
  online) run_online ;;
  offline) run_offline ;;
  offline-runtime-versions) python3 scripts/offline_runtime_versions.py --check ;;
  offline-voice-e2e-report) bash scripts/smoke_test_offline_voice_real.sh ;;
  offline-showcase-report)
    REPORT_ARGS=()
    if [[ "${OFFLINE_SHOWCASE_RUN_LATENCY:-false}" == "true" ]]; then
      REPORT_ARGS+=(--run-latency)
    fi
    if [[ "${OFFLINE_SHOWCASE_RUN_LLAMA_BENCH:-false}" == "true" ]]; then
      REPORT_ARGS+=(--run-llama-bench)
    fi
    if [[ -n "${OFFLINE_SHOWCASE_LLAMA_BENCH_INPUT:-}" ]]; then
      REPORT_ARGS+=(--llama-bench-input "$OFFLINE_SHOWCASE_LLAMA_BENCH_INPUT")
    fi
    if [[ -n "${OFFLINE_SHOWCASE_LLAMA_BENCH_OUTPUT:-}" ]]; then
      REPORT_ARGS+=(--llama-bench-output "$OFFLINE_SHOWCASE_LLAMA_BENCH_OUTPUT")
    fi
    if [[ "${OFFLINE_SHOWCASE_RUN_INSTRUCTION_FOLLOWING:-false}" == "true" ]]; then
      REPORT_ARGS+=(--run-instruction-following)
    fi
    if [[ -n "${OFFLINE_SHOWCASE_INSTRUCTION_FOLLOWING_INPUT:-}" ]]; then
      REPORT_ARGS+=(--instruction-following-input "$OFFLINE_SHOWCASE_INSTRUCTION_FOLLOWING_INPUT")
    fi
    if [[ -n "${OFFLINE_SHOWCASE_INSTRUCTION_FOLLOWING_OUTPUT:-}" ]]; then
      REPORT_ARGS+=(--instruction-following-output "$OFFLINE_SHOWCASE_INSTRUCTION_FOLLOWING_OUTPUT")
    fi
    if [[ "${OFFLINE_SHOWCASE_RUN_ASR_TTS:-false}" == "true" ]]; then
      REPORT_ARGS+=(--run-asr-tts)
    fi
    if [[ "${OFFLINE_SHOWCASE_RUN_VOICE_E2E:-false}" == "true" ]]; then
      REPORT_ARGS+=(--run-voice-e2e)
    fi
    if [[ -n "${OFFLINE_SHOWCASE_VOICE_E2E_INPUT:-}" ]]; then
      REPORT_ARGS+=(--voice-e2e-input "$OFFLINE_SHOWCASE_VOICE_E2E_INPUT")
    fi
    if [[ -n "${OFFLINE_SHOWCASE_VOICE_E2E_OUTPUT:-}" ]]; then
      REPORT_ARGS+=(--voice-e2e-output "$OFFLINE_SHOWCASE_VOICE_E2E_OUTPUT")
    fi
    python3 scripts/generate_offline_showcase_report.py "${REPORT_ARGS[@]}"
    ;;
  offline-evidence-audit)
    if [[ ! -s "${OFFLINE_EVIDENCE_REPORT:-logs/offline_showcase_report.json}" ]]; then
      python3 scripts/generate_offline_showcase_report.py
    fi
    python3 scripts/audit_offline_showcase_evidence.py \
      --input "${OFFLINE_EVIDENCE_REPORT:-logs/offline_showcase_report.json}" \
      --output "${OFFLINE_EVIDENCE_AUDIT_OUTPUT:-logs/offline_evidence_audit.json}" \
      ${OFFLINE_EVIDENCE_REQUIRE_LATENCY:+--require-latency} \
      ${OFFLINE_EVIDENCE_REQUIRE_LLAMA_BENCH:+--require-llama-bench} \
      ${OFFLINE_EVIDENCE_REQUIRE_INSTRUCTION_FOLLOWING:+--require-instruction-following} \
      ${OFFLINE_EVIDENCE_REQUIRE_ASR_TTS:+--require-asr-tts} \
      ${OFFLINE_EVIDENCE_REQUIRE_VOICE_E2E:+--require-voice-e2e}
    ;;
  offline-latency) check_llama_cpp_runtime; bash scripts/smoke_test_offline_latency.sh ;;
  llama-cpp-preflight) check_llama_cpp_runtime; bash scripts/smoke_test_llama_cpp.sh preflight ;;
  llama-cpp-smoke) check_llama_cpp_runtime; bash scripts/smoke_test_llama_cpp.sh smoke ;;
  llama-decode-benchmark)
    check_llama_cpp_runtime
    require_file third_party/llama.cpp/build/bin/llama-bench
    python3 scripts/benchmark_llama_decode_speed.py \
      --minimum-decode-tokens-per-s "${LLAMA_DECODE_MIN_TOKENS_PER_S:-8.6}" \
      ${LLAMA_BENCH_NO_WARMUP:+--no-warmup}
    ;;
  summer-tts-preflight) check_summer_tts_runtime; python3 scripts/summer_tts_smoke.py --preflight-only ;;
  summer-tts-smoke) check_summer_tts_runtime; python3 scripts/summer_tts_smoke.py ;;
  summer-pseudo-tts) check_summer_tts_runtime; python3 scripts/smoke_test_summer_pseudo_tts.py ;;
  summer-tts-service) bash scripts/smoke_test_summer_tts_service.sh ;;
  summer-tts-cache-audit)
    python3 scripts/audit_summer_tts_cache_evidence.py \
      --input "${SUMMER_TTS_CACHE_PROBE_REPORT:-logs/summer_tts_service_probe.json}" \
      --output "${SUMMER_TTS_CACHE_AUDIT_REPORT:-logs/summer_tts_cache_audit.json}" \
      --target-roundtrip-ms "${SUMMER_TTS_CACHE_TARGET_MS:-300}" \
      --min-pcm-bytes "${SUMMER_TTS_CACHE_MIN_PCM_BYTES:-1000}" \
      ${SUMMER_TTS_CACHE_ALLOW_MISS:+--allow-cache-miss}
    ;;
  pseudo-tts) python3 scripts/smoke_test_pseudo_streaming_tts.py ;;
  sherpa-asr-preflight) run_sherpa_asr_preflight ;;
  sherpa-asr-smoke) run_sherpa_asr_smoke ;;
  offline-sherpa-typed) check_offline_runtime; bash scripts/smoke_test_offline_sherpa_typed_simulation.sh ;;
  demo) bash scripts/smoke_test_demo_sequence.sh ;;
  navigation-demo) bash scripts/smoke_test_navigation_sequence.sh online; bash scripts/smoke_test_navigation_sequence.sh offline ;;
  nav2-bridge) bash scripts/smoke_test_nav2_bridge.sh ;;
  nav2-preflight) bash scripts/smoke_test_nav2_preflight.sh ;;
  nav2-assets) python3 scripts/audit_nav2_demo_assets.py ;;
  nav2-stage)
    run_isolated_ros_smoke 181 bash scripts/smoke_test_navigation_sequence.sh online
    run_isolated_ros_smoke 182 bash scripts/smoke_test_navigation_sequence.sh offline
    run_isolated_ros_smoke 183 bash scripts/smoke_test_continuous_navigation_queue.sh online
    run_isolated_ros_smoke 184 bash scripts/smoke_test_continuous_navigation_queue.sh offline
    run_isolated_ros_smoke 185 bash scripts/smoke_test_continuous_navigation_natural.sh online
    run_isolated_ros_smoke 186 bash scripts/smoke_test_continuous_navigation_natural.sh offline
    run_isolated_ros_smoke 187 bash scripts/smoke_test_nav2_bridge.sh
    bash scripts/smoke_test_nav2_preflight.sh
    ;;
  nav2-turtlebot3) bash scripts/smoke_test_nav2_turtlebot3_voice.sh ;;
  nav2-resilience)
    NAV2_RESILIENCE=true SKIP_PATROL=1 \
      bash scripts/smoke_test_nav2_turtlebot3_voice.sh
    ;;
  mapping-stage)
    python3 scripts/audit_slam_mapping_assets.py
    colcon build --packages-select embodied_slam --symlink-install
    colcon test --packages-select embodied_slam --event-handlers console_direct+
    colcon test-result --test-result-base build/embodied_slam --verbose
    ;;
  slam-nav-showcase-stage)
    embodied_workspace_doctor true
    python3 scripts/generate_showcase_scene.py --check
    python3 -m pytest -q tests/repository/test_showcase_scene.py
    PYTHONPATH="$WORKSPACE/src/embodied_agent_core${PYTHONPATH:+:$PYTHONPATH}" \
      python3 -m pytest -q src/embodied_agent_core/test/test_command_nlu.py
    WORKSPACE="$WORKSPACE" bash scripts/voice_slam_nav_showcase.sh audit
    ;;
  slam-nav-showcase) bash scripts/smoke_test_slam_nav_showcase.sh ;;
  slam-nav-showcase-mapping) bash scripts/smoke_test_slam_nav_showcase_mapping.sh ;;
  slam-session-orchestrator-stage)
    PYTHONPATH="$WORKSPACE/src/embodied_slam_tools${PYTHONPATH:+:$PYTHONPATH}" \
      python3 -m pytest -q src/embodied_slam_tools/test/test_showcase_session.py
    colcon build --symlink-install --packages-up-to embodied_slam_tools \
      --allow-overriding embodied_agent_interfaces embodied_slam_tools
    WORKSPACE="$WORKSPACE" bash scripts/smoke_test_voice_slam_session_orchestrator.sh
    ;;
  slam-autonomous-mission-stage)
    PYTHONPATH="$WORKSPACE/src/embodied_slam_tools${PYTHONPATH:+:$PYTHONPATH}" \
      python3 -m pytest -q src/embodied_slam_tools/test/test_showcase_session.py
    colcon build --symlink-install --packages-up-to embodied_slam_tools \
      --allow-overriding embodied_agent_interfaces embodied_slam_tools
    PYTHONPATH="$WORKSPACE/src/embodied_slam_tools${PYTHONPATH:+:$PYTHONPATH}" \
      python3 -m pytest -q \
        src/embodied_slam_tools/test/test_showcase_session_node.py \
        tests/integration/test_trigger_automatic_slam_mission.py
    WORKSPACE="$WORKSPACE" bash scripts/smoke_test_voice_slam_automatic_mission.sh
    WORKSPACE="$WORKSPACE" bash scripts/smoke_test_voice_slam_automatic_cancel.sh
    ;;
  slam-autonomous-mission)
    embodied_workspace_doctor true
    WORKSPACE="$WORKSPACE" bash scripts/smoke_test_voice_slam_automatic_mission_gazebo.sh
    ;;
  slam-session-orchestrator)
    bash scripts/smoke_test_voice_slam_session_orchestrator_gazebo.sh
    ;;
  slam-benchmark) bash scripts/smoke_test_slam_mapping_baseline.sh ;;
  slam-gtsam-benchmark) SLAM_SOLVER=gtsam bash scripts/smoke_test_slam_mapping_baseline.sh ;;
  slam-ab-benchmark)
    SLAM_SOLVER=ceres bash scripts/smoke_test_slam_mapping_baseline.sh
    SLAM_SOLVER=gtsam bash scripts/smoke_test_slam_mapping_baseline.sh
    python3 scripts/compare_slam_backends.py
    ;;
  slam-navigation) bash scripts/smoke_test_slam_localization_navigation.sh ;;
  slam-evaluation-stage) run_slam_evaluation_stage ;;
  openloris-groundtruth)
    python3 scripts/setup_openloris_groundtruth.py \
      --sequence "${OPENLORIS_SEQUENCE:-office1-1}" \
      --output-root "${OPENLORIS_ROOT:-datasets/openloris}"
    ;;
  openloris-rosbag-setup)
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
    python3 scripts/setup_openloris_rosbag.py "${ROSBAG_ARGS[@]}"
    ;;
  openloris-sequence-ranking)
    OPENLORIS_ROOT="${OPENLORIS_ROOT:-$WORKSPACE/datasets/openloris}"
    python3 scripts/setup_openloris_groundtruth.py \
      --sequence "${OPENLORIS_SEQUENCE:-office1-1}" --output-root "$OPENLORIS_ROOT"
    python3 scripts/rank_openloris_revisit_sequences.py \
      --archive "$OPENLORIS_ROOT/archive/groundtruth.zip" \
      --sensor-contracts "$WORKSPACE/src/embodied_slam/config/openloris_sensor_contracts.json" \
      --output "${OPENLORIS_RANKING_REPORT:-logs/openloris/revisit_sequence_ranking.json}"
    ;;
  openloris-long-loop-evidence)
    bash scripts/run_openloris_long_loop_evidence.sh
    ;;
  openloris-robust-kernel-ablation)
    bash scripts/run_gtsam_robust_kernel_ablation.sh
    ;;
  openloris-loop-consistency-ablation)
    bash scripts/run_gtsam_loop_consistency_ablation.sh
    ;;
  openloris-scan-overlap-ablation)
    bash scripts/run_gtsam_scan_overlap_ablation.sh
    ;;
  openloris-scan-overlap-multisequence)
    bash scripts/run_gtsam_scan_overlap_multisequence.sh
    ;;
  openloris-lidar-loop-candidates)
    bash scripts/run_openloris_lidar_loop_candidates.sh
    ;;
  openloris-lidar-shadow-matches)
    bash scripts/run_openloris_lidar_shadow_matches.sh
    ;;
  openloris-lidar-submap-ablation)
    bash scripts/run_openloris_lidar_submap_ablation.sh
    ;;
  lidar-loop-runtime)
    colcon build --packages-up-to embodied_slam --symlink-install
    bash scripts/smoke_test_lidar_loop_runtime.sh
    ;;
  openloris-replay-stage)
    python3 -c 'import rosbags' || {
      echo "Missing optional replay runtime; run: pip install -r requirements-slam-eval.txt" >&2
      exit 2
    }
    colcon build --packages-up-to embodied_slam embodied_slam_tools --symlink-install
    colcon test --packages-select embodied_slam embodied_slam_tools --event-handlers console_direct+
    colcon test-result --test-result-base build/embodied_slam --verbose
    colcon test-result --test-result-base build/embodied_slam_tools --verbose
    python3 -m pytest -q \
      tests/repository/test_openloris_backend_comparison.py \
      tests/repository/test_openloris_rosbag_setup.py \
      tests/repository/test_openloris_experiment_manifest.py \
      tests/repository/test_slam_degradation_analysis.py
    bash scripts/smoke_test_openloris_replay_adapter.sh
    ;;
  openloris-bag-preflight)
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
    ;;
  openloris-slam-ceres) bash scripts/run_openloris_slam_replay.sh ceres ;;
  openloris-slam-gtsam) bash scripts/run_openloris_slam_replay.sh gtsam ;;
  openloris-slam-ab)
    bash scripts/run_openloris_slam_replay.sh ceres
    bash scripts/run_openloris_slam_replay.sh gtsam
    OPENLORIS_SEQUENCE="${OPENLORIS_SEQUENCE:-office1-1}"
    OPENLORIS_OUTPUT_DIR="${OPENLORIS_OUTPUT_DIR:-$WORKSPACE/logs/openloris/$OPENLORIS_SEQUENCE}"
    python3 scripts/compare_openloris_backends.py \
      --ceres "$OPENLORIS_OUTPUT_DIR/ceres_report.json" \
      --gtsam "$OPENLORIS_OUTPUT_DIR/gtsam_report.json" \
      --ceres-degradation "$OPENLORIS_OUTPUT_DIR/ceres_degradation.json" \
      --gtsam-degradation "$OPENLORIS_OUTPUT_DIR/gtsam_degradation.json" \
      --output "$OPENLORIS_OUTPUT_DIR/backend_comparison.json"
    ;;
  openloris-loop-evidence)
    OPENLORIS_SEQUENCE="${OPENLORIS_SEQUENCE:-office1-7}"
    OPENLORIS_ROOT="${OPENLORIS_ROOT:-$WORKSPACE/datasets/openloris}"
    OPENLORIS_REFERENCE="$OPENLORIS_ROOT/groundtruth/$OPENLORIS_SEQUENCE/groundtruth.txt"
    if [[ ! -s "$OPENLORIS_REFERENCE" ]]; then
      python3 scripts/setup_openloris_groundtruth.py \
        --sequence "$OPENLORIS_SEQUENCE" --output-root "$OPENLORIS_ROOT"
    fi
    python3 scripts/analyze_openloris_revisits.py \
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
      bash scripts/run_openloris_slam_replay.sh gtsam
    ;;
  openloris-loop-sweep)
    colcon build --packages-up-to embodied_slam embodied_slam_tools --symlink-install
    bash scripts/run_openloris_loop_sweep.sh
    ;;
  openloris-evaluate)
    if [[ -z "${SLAM_ESTIMATE_FILE:-}" ]]; then
      echo "Usage: SLAM_ESTIMATE_FILE=/path/to/estimate.tum $0 openloris-evaluate" >&2
      exit 2
    fi
    OPENLORIS_SEQUENCE="${OPENLORIS_SEQUENCE:-office1-1}"
    OPENLORIS_ROOT="${OPENLORIS_ROOT:-datasets/openloris}"
    OPENLORIS_REFERENCE="$OPENLORIS_ROOT/groundtruth/$OPENLORIS_SEQUENCE/groundtruth.txt"
    if [[ ! -s "$OPENLORIS_REFERENCE" ]]; then
      python3 scripts/setup_openloris_groundtruth.py \
        --sequence "$OPENLORIS_SEQUENCE" --output-root "$OPENLORIS_ROOT"
    fi
    python3 scripts/evaluate_slam_trajectory.py \
      --reference "$OPENLORIS_REFERENCE" \
      --estimate "$SLAM_ESTIMATE_FILE" \
      --output "${SLAM_EVALUATION_REPORT:-logs/openloris_${OPENLORIS_SEQUENCE}_report.json}" \
      --max-time-diff "${SLAM_MAX_TIME_DIFF_S:-0.05}" \
      --rpe-delta "${SLAM_RPE_DELTA_S:-1.0}" \
      --min-match-ratio "${SLAM_MIN_MATCH_RATIO:-0.80}" \
      ${SLAM_MAX_ATE_RMSE_M:+--max-ate-rmse "$SLAM_MAX_ATE_RMSE_M"} \
      ${SLAM_MAX_RPE_RMSE_M:+--max-rpe-translation-rmse "$SLAM_MAX_RPE_RMSE_M"}
    ;;
  dynamic-obstacle-stage)
    python3 scripts/build_slam_nav2_params.py --output logs/slam_nav2_params.yaml
    colcon build --packages-up-to embodied_navigation --symlink-install --allow-overriding \
      embodied_agent_interfaces embodied_navigation
    colcon test --packages-select embodied_navigation --event-handlers console_direct+
    colcon test-result --test-result-base build/embodied_navigation --verbose
    set +u
    source install/setup.bash
    set -u
    bash scripts/smoke_test_dynamic_obstacle_tracker.sh
    run_dynamic_obstacle_ablation
    ;;
  dynamic-obstacle-ablation)
    colcon build --packages-up-to embodied_navigation --symlink-install --allow-overriding \
      embodied_agent_interfaces embodied_navigation
    set +u
    source install/setup.bash
    set -u
    run_dynamic_obstacle_ablation
    ;;
  dynamic-obstacle-navigation) bash scripts/smoke_test_predicted_dynamic_obstacle_navigation.sh ;;
  dynamic-obstacle-navigation-ablation) bash scripts/run_dynamic_obstacle_navigation_ablation.sh ;;
  continuous-mock) bash scripts/smoke_test_continuous_voice.sh online; bash scripts/smoke_test_continuous_voice.sh offline ;;
  continuous-soak) bash scripts/smoke_test_continuous_voice_soak.sh online; bash scripts/smoke_test_continuous_voice_soak.sh offline ;;
  continuous-endpoint) bash scripts/smoke_test_continuous_endpoint_asr.sh online; bash scripts/smoke_test_continuous_endpoint_asr.sh offline ;;
  continuous-multi-command) bash scripts/smoke_test_continuous_multi_command.sh online; bash scripts/smoke_test_continuous_multi_command.sh offline ;;
  continuous-navigation) bash scripts/smoke_test_continuous_navigation_queue.sh online; bash scripts/smoke_test_continuous_navigation_queue.sh offline ;;
  continuous-navigation-natural) bash scripts/smoke_test_continuous_navigation_natural.sh online; bash scripts/smoke_test_continuous_navigation_natural.sh offline ;;
  continuous-queue-full) bash scripts/smoke_test_continuous_queue_full.sh online; bash scripts/smoke_test_continuous_queue_full.sh offline ;;
  continuous-ttl) bash scripts/smoke_test_continuous_command_ttl.sh online; bash scripts/smoke_test_continuous_command_ttl.sh offline ;;
  continuous-timeout) bash scripts/smoke_test_continuous_session_timeout.sh online; bash scripts/smoke_test_continuous_session_timeout.sh offline ;;
  continuous-kws-mock) bash scripts/smoke_test_continuous_kws_sidecar.sh online; bash scripts/smoke_test_continuous_kws_sidecar.sh offline ;;
  speaker-memory-mock) bash scripts/smoke_test_speaker_memory.sh online; bash scripts/smoke_test_speaker_memory.sh offline ;;
  speaker-enroll) bash scripts/smoke_test_speaker_enrollment.sh ;;
  speaker-runtime) source "$WORKSPACE/scripts/activate.sh"; python3 scripts/probe_sherpa_speaker_runtime.py; bash scripts/smoke_test_sherpa_speaker_identity.sh ;;
  vad-sidecar) bash scripts/smoke_test_silero_vad_sidecar.sh ;;
  silero-vad-runtime)
    python3 scripts/silero_onnx_smoke.py \
      --model "${SILERO_VAD_MODEL_PATH:-models/silero_vad/silero_vad.onnx}" \
      --output "${SILERO_VAD_REPORT:-logs/silero_vad_runtime.json}"
    bash scripts/smoke_test_silero_vad_runtime.sh
    ;;
  webrtc-vad-sidecar) bash scripts/smoke_test_webrtc_vad_sidecar.sh ;;
  kws-sidecar) bash scripts/smoke_test_keyword_wake_sidecar.sh ;;
  sherpa-kws-sidecar) bash scripts/smoke_test_sherpa_kws_sidecar.sh ;;
  openwakeword-sidecar) bash scripts/smoke_test_openwakeword_sidecar.sh ;;
  livekit-sidecar) bash scripts/smoke_test_livekit_wakeword_sidecar.sh ;;
  kws-calibration) bash scripts/smoke_test_kws_score_calibration.sh ;;
  voice-readiness) bash scripts/smoke_test_voice_readiness.sh ;;
  provider-preflight)
    python3 -m pytest -q tests/integration/test_voice_provider_preflight.py
    python3 scripts/voice_provider_preflight.py \
      --mode "${PROVIDER_PREFLIGHT_MODE:-offline}" \
      --vad-provider "${VAD_PROVIDER:-auto}" \
      --kws-provider "${KWS_PROVIDER:-none}" \
      --silero-model-path "${SILERO_VAD_MODEL_PATH:-$WORKSPACE/models/silero_vad/silero_vad.onnx}" \
      --silero-use-onnx "${SILERO_VAD_USE_ONNX:-true}" \
      --sherpa-tokens "${SHERPA_KWS_TOKENS:-}" \
      --sherpa-encoder "${SHERPA_KWS_ENCODER:-}" \
      --sherpa-decoder "${SHERPA_KWS_DECODER:-}" \
      --sherpa-joiner "${SHERPA_KWS_JOINER:-}" \
      --sherpa-keywords-file "${SHERPA_KWS_KEYWORDS_FILE:-}" \
      --openwakeword-models "${OPENWAKEWORD_MODELS:-}" \
      --livekit-wakeword-models "${LIVEKIT_WAKEWORD_MODELS:-}"
    ;;
  voice-stability-preflight)
    set +e
    STABILITY_OUTPUT="$(python3 scripts/voice_provider_preflight.py \
      --mode "${PROVIDER_PREFLIGHT_MODE:-offline}" \
      --vad-provider "${VAD_PROVIDER:-auto}" \
      --kws-provider "${KWS_PROVIDER:-none}" \
      --silero-model-path "${SILERO_VAD_MODEL_PATH:-$WORKSPACE/models/silero_vad/silero_vad.onnx}" \
      --silero-use-onnx "${SILERO_VAD_USE_ONNX:-true}" \
      --sherpa-tokens "${SHERPA_KWS_TOKENS:-}" \
      --sherpa-encoder "${SHERPA_KWS_ENCODER:-}" \
      --sherpa-decoder "${SHERPA_KWS_DECODER:-}" \
      --sherpa-joiner "${SHERPA_KWS_JOINER:-}" \
      --sherpa-keywords-file "${SHERPA_KWS_KEYWORDS_FILE:-}" \
      --openwakeword-models "${OPENWAKEWORD_MODELS:-}" \
      --livekit-wakeword-models "${LIVEKIT_WAKEWORD_MODELS:-}" \
      --require-mature-vad \
      --json)"
    STABILITY_STATUS=$?
    set -e
    printf '%s\n' "$STABILITY_OUTPUT"
    if [[ -n "${VOICE_STABILITY_REPORT:-logs/voice_stability_preflight.json}" ]]; then
      mkdir -p "$(dirname "${VOICE_STABILITY_REPORT:-logs/voice_stability_preflight.json}")"
      printf '%s\n' "$STABILITY_OUTPUT" > "${VOICE_STABILITY_REPORT:-logs/voice_stability_preflight.json}"
    fi
    exit "$STABILITY_STATUS"
    ;;
  voice-vad-runtime-dry-run)
    bash scripts/setup_voice_vad_runtime.sh "${VOICE_VAD_PROFILE:-all}" --dry-run
    ;;
  voice-kws-runtime-dry-run)
    bash scripts/setup_voice_kws_runtime.sh "${VOICE_KWS_PROFILE:-all}" --dry-run
    ;;
  voice-calibration-report)
    if [[ "${VOICE_CALIBRATION_COLLECT:-false}" == "true" ]]; then
      python3 scripts/voice_calibration_report.py \
        --collect \
        --duration "${VOICE_CALIBRATION_DURATION:-6}" \
        --mode "${VOICE_CALIBRATION_MODE:-offline}" \
        --vad-provider "${VAD_PROVIDER:-auto}" \
        --kws-provider "${KWS_PROVIDER:-none}"
    else
      python3 scripts/voice_calibration_report.py \
        --synthetic-profile "${VOICE_CALIBRATION_SYNTHETIC_PROFILE:-low_gain}" \
        --mode "${VOICE_CALIBRATION_MODE:-offline}" \
        --vad-provider "${VAD_PROVIDER:-auto}" \
        --kws-provider "${KWS_PROVIDER:-none}"
    fi
    ;;
  instruction-eval-dataset) python3 scripts/validate_instruction_eval_dataset.py ;;
  instruction-parser-eval) python3 scripts/evaluate_instruction_parser.py --minimum "${INSTRUCTION_PARSER_MINIMUM:-1.0}" ;;
  instruction-following-eval)
    check_llama_cpp_runtime
    bash scripts/evaluate_instruction_following.sh \
      --minimum "${INSTRUCTION_FOLLOWING_MINIMUM:-0.0}" \
      --minimum-effective "${INSTRUCTION_FOLLOWING_EFFECTIVE_MINIMUM:-0.0}"
    ;;
  instruction-following-lora-candidates)
    if [[ ! -s "${INSTRUCTION_FOLLOWING_REPORT:-logs/instruction_following_report.json}" ]]; then
      echo "Missing instruction following report; run: bash scripts/acceptance_test.sh instruction-following-eval" >&2
      exit 1
    fi
    python3 scripts/export_instruction_following_lora_candidates.py \
      --report "${INSTRUCTION_FOLLOWING_REPORT:-logs/instruction_following_report.json}" \
      --dataset "${INSTRUCTION_FOLLOWING_DATASET:-training/robot_dialogue_seed.jsonl}" \
      --output "${INSTRUCTION_FOLLOWING_LORA_CANDIDATES:-training/robot_dialogue_lora_candidates.jsonl}" \
      --metadata-output "${INSTRUCTION_FOLLOWING_LORA_CANDIDATES_META:-training/robot_dialogue_lora_candidates.meta.json}" \
      ${INSTRUCTION_FOLLOWING_LORA_FAIL_IF_EMPTY:+--fail-if-empty}
    ;;
  instruction-following-lora-review)
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
    python3 scripts/review_lora_candidates.py "${LORA_REVIEW_ARGS[@]}"
    ;;
  lora-q8-pipeline) bash scripts/build_qwen_lora_q8.sh --dry-run ;;
  lora-q8-comparison)
    bash scripts/evaluate_lora_q8_comparison.sh
    python3 scripts/audit_lora_q8_pipeline.py --strict-reproduced
    ;;
  asr-nlu-samples-to-eval)
    ASR_NLU_EVAL_OUTPUT="${ASR_NLU_EVAL_OUTPUT:-logs/asr_nlu_eval_candidates.jsonl}"
    if [[ "${ASR_NLU_SAMPLES_SYNTHETIC:-true}" == "true" ]]; then
      python3 scripts/asr_nlu_samples_to_eval_candidates.py \
        --synthetic-demo \
        --output "$ASR_NLU_EVAL_OUTPUT"
    else
      python3 scripts/asr_nlu_samples_to_eval_candidates.py \
        --input "${ASR_NLU_SAMPLE_LOG:-logs/asr_nlu_samples.jsonl}" \
        --output "$ASR_NLU_EVAL_OUTPUT"
    fi
    ;;
  asr-nlu-candidate-eval)
    ASR_NLU_CANDIDATE_INPUT="${ASR_NLU_CANDIDATE_INPUT:-logs/asr_nlu_eval_candidates.synthetic.jsonl}"
    if [[ "${ASR_NLU_CANDIDATE_SYNTHETIC:-true}" == "true" ]]; then
      python3 scripts/asr_nlu_samples_to_eval_candidates.py \
        --synthetic-demo \
        --output "$ASR_NLU_CANDIDATE_INPUT"
    fi
    python3 scripts/evaluate_asr_nlu_eval_candidates.py \
      --input "$ASR_NLU_CANDIDATE_INPUT" \
      --output "${ASR_NLU_CANDIDATE_REPORT:-logs/asr_nlu_candidate_eval_report.json}" \
      --minimum "${ASR_NLU_CANDIDATE_MINIMUM:-1.0}"
    ;;
  release-gate) python3 scripts/showcase_release_gate.py --workspace "$WORKSPACE" ;;
  robotics-gate) python3 scripts/showcase_release_gate.py --workspace "$WORKSPACE" --profile robotics ;;
  demo-gate) python3 scripts/showcase_release_gate.py --workspace "$WORKSPACE" --profile demo ;;
  demo-evidence-checklist)
    CHECKLIST_ARGS=(
      --output "${DEMO_EVIDENCE_REPORT:-logs/demo_evidence_checklist.json}"
      --markdown "${DEMO_EVIDENCE_MARKDOWN:-logs/demo_evidence_checklist.md}"
      --automatic-report "${DEMO_EVIDENCE_AUTOMATIC_REPORT:-logs/demo_acceptance_report.json}"
      --voice-stability-report "${DEMO_EVIDENCE_VOICE_STABILITY_REPORT:-logs/voice_stability_preflight.json}"
      --voice-report "${DEMO_EVIDENCE_VOICE_REPORT:-logs/continuous-live-check.json}"
      --nav2-report "${DEMO_EVIDENCE_NAV2_REPORT:-logs/nav2-live-check.json}"
      --recording "${DEMO_EVIDENCE_RECORDING:-logs/demo_recording.mp4}"
      --screenshot "${DEMO_EVIDENCE_SCREENSHOT:-logs/demo_screenshot.png}"
    )
    if [[ "${DEMO_EVIDENCE_REQUIRE_NAV2:-false}" == "true" ]]; then
      CHECKLIST_ARGS+=(--require-nav2)
    fi
    if [[ "${DEMO_EVIDENCE_REQUIRE_VISUAL:-false}" == "true" ]]; then
      CHECKLIST_ARGS+=(--require-visual-evidence)
    fi
    if [[ "${DEMO_EVIDENCE_STRICT:-false}" == "true" ]]; then
      CHECKLIST_ARGS+=(--strict)
    fi
    python3 scripts/demo_evidence_checklist.py "${CHECKLIST_ARGS[@]}"
    ;;
  wsl-microphone-preflight) bash scripts/wsl_microphone_preflight.sh ;;
  gazebo) run_gazebo ;;
  cpp-action-client) bash scripts/smoke_test_cpp_action_client.sh ;;
  cpp-action-scheduler) bash scripts/smoke_test_cpp_action_scheduler.sh ;;
  cpp-action-bridge-lifecycle) bash scripts/smoke_test_typed_action_bridge_lifecycle.sh ;;
  gazebo-voice) check_offline_runtime; USE_TYPED_ACTIONS=true bash scripts/smoke_test_gazebo_voice.sh ;;
  gazebo-voice-online) bash scripts/smoke_test_gazebo_voice_online.sh ;;
  microphone-offline) bash scripts/accept_voice_simulation_microphone.sh offline ;;
  microphone-online) bash scripts/accept_voice_simulation_microphone.sh online ;;
  continuous-offline) bash scripts/continuous_voice_control.sh offline ;;
  continuous-online) bash scripts/continuous_voice_control.sh online ;;
  continuous-nav2-offline) bash scripts/continuous_nav2_voice_control.sh offline ;;
  continuous-nav2-online) bash scripts/continuous_nav2_voice_control.sh online ;;
  voice-slam-workplace-demo)
    CHECK_MODE="${2:-offline}"
    if [[ "$CHECK_MODE" != "offline" && "$CHECK_MODE" != "online" ]]; then
      echo "Usage: $0 voice-slam-workplace-demo {offline|online}" >&2
      exit 2
    fi
    WORKSPACE="$WORKSPACE" bash scripts/voice_slam_nav_showcase.sh auto "$CHECK_MODE"
    ;;
  continuous-nav2-evidence)
    CHECK_MODE="${2:-offline}"
    if [[ "$CHECK_MODE" != "offline" && "$CHECK_MODE" != "online" ]]; then
      echo "Usage: $0 continuous-nav2-evidence {offline|online}" >&2
      exit 2
    fi
    bash scripts/continuous_nav2_voice_evidence.sh "$CHECK_MODE"
    ;;
  continuous-voice-evidence)
    CHECK_MODE="${2:-offline}"
    if [[ "$CHECK_MODE" != "offline" && "$CHECK_MODE" != "online" ]]; then
      echo "Usage: $0 continuous-voice-evidence {offline|online}" >&2
      exit 2
    fi
    bash scripts/continuous_voice_evidence.sh "$CHECK_MODE"
    ;;
  continuous-live-check)
    CHECK_MODE="${2:-offline}"
    if [[ "$CHECK_MODE" != "offline" && "$CHECK_MODE" != "online" ]]; then
      echo "Usage: $0 continuous-live-check {offline|online}" >&2
      exit 2
    fi
    echo "continuous-live-check=$CHECK_MODE：请先在另一个终端启动 acceptance_test.sh continuous-$CHECK_MODE"
    LIVE_CHECK_ARGS=(
      --agent-mode "$CHECK_MODE"
      --duration "${CONTINUOUS_LIVE_CHECK_DURATION:-180}"
    )
    if [[ -n "${CONTINUOUS_LIVE_CHECK_REPORT:-}" ]]; then
      LIVE_CHECK_ARGS+=(--output "$CONTINUOUS_LIVE_CHECK_REPORT")
    fi
    python3 scripts/continuous_live_check.py "${LIVE_CHECK_ARGS[@]}"
    ;;
  continuous-voice-benchmark)
    CHECK_MODE="${2:-offline}"
    if [[ "$CHECK_MODE" != "offline" && "$CHECK_MODE" != "online" ]]; then
      echo "Usage: $0 continuous-voice-benchmark {offline|online}" >&2
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
    if python3 scripts/evaluate_live_voice_benchmark.py \
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
    ;;
  continuous-nav2-live-check)
    CHECK_MODE="${2:-offline}"
    if [[ "$CHECK_MODE" != "offline" && "$CHECK_MODE" != "online" ]]; then
      echo "Usage: $0 continuous-nav2-live-check {offline|online}" >&2
      exit 2
    fi
    echo "continuous-nav2-live-check=$CHECK_MODE：请先在另一个终端启动 acceptance_test.sh continuous-nav2-$CHECK_MODE"
    LIVE_CHECK_ARGS=(
      --scenario nav2 \
      --agent-mode "$CHECK_MODE" \
      --duration "${CONTINUOUS_LIVE_CHECK_DURATION:-240}" \
      --min-asr "${CONTINUOUS_NAV2_LIVE_MIN_ASR:-4}" \
      --min-candidates "${CONTINUOUS_NAV2_LIVE_MIN_CANDIDATES:-2}" \
      --min-success "${CONTINUOUS_NAV2_LIVE_MIN_SUCCESS:-2}" \
      --require-candidate navigate_to \
      --require-candidate follow_waypoints \
      --require-navigation-details
    )
    if [[ -n "${CONTINUOUS_LIVE_CHECK_REPORT:-}" ]]; then
      LIVE_CHECK_ARGS+=(--output "$CONTINUOUS_LIVE_CHECK_REPORT")
    fi
    python3 scripts/continuous_live_check.py "${LIVE_CHECK_ARGS[@]}"
    ;;
  continuous-live-report)
    REPORT_PATH="${2:-}"
    if [[ -z "$REPORT_PATH" ]]; then
      echo "Usage: $0 continuous-live-report REPORT_FILE" >&2
      exit 2
    fi
    python3 scripts/continuous_live_check.py --input-report "$REPORT_PATH"
    ;;
  runtime-evidence-summary)
    python3 scripts/generate_runtime_evidence_summary.py
    ;;
  voice-benchmark-report)
    REPORT_PATH="${2:-}"
    if [[ -z "$REPORT_PATH" ]]; then
      echo "Usage: $0 voice-benchmark-report REPORT_FILE" >&2
      exit 2
    fi
    python3 scripts/evaluate_live_voice_benchmark.py --report "$REPORT_PATH"
    ;;
  continuous-nav2-live-report)
    REPORT_PATH="${2:-}"
    if [[ -z "$REPORT_PATH" ]]; then
      echo "Usage: $0 continuous-nav2-live-report REPORT_FILE" >&2
      exit 2
    fi
    python3 scripts/continuous_live_check.py \
      --scenario nav2 \
      --input-report "$REPORT_PATH" \
      --min-asr "${CONTINUOUS_NAV2_LIVE_MIN_ASR:-4}" \
      --min-candidates "${CONTINUOUS_NAV2_LIVE_MIN_CANDIDATES:-2}" \
      --min-success "${CONTINUOUS_NAV2_LIVE_MIN_SUCCESS:-2}" \
      --require-candidate navigate_to \
      --require-candidate follow_waypoints \
      --require-navigation-details
    ;;
  all) run_base; run_online; run_offline; bash scripts/smoke_test_demo_sequence.sh; run_gazebo; USE_TYPED_ACTIONS=true bash scripts/smoke_test_gazebo_voice.sh; bash scripts/smoke_test_gazebo_voice_online.sh ;;
  *) usage >&2; exit 2 ;;
esac
