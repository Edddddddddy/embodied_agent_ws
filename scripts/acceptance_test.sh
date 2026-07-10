#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
LEVEL="${1:-mock}"

usage() {
  cat <<'EOF'
Usage: acceptance_test.sh MODE

Automated modes:
  core                Typical developer gate: repository, Python unit, C++ unit tests
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
  offline-latency     Check llama.cpp first token <1s and default TTS first audio <300ms
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
  vad-sidecar         Dependency-free Silero VAD sidecar seam smoke test
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
  asr-nlu-samples-to-eval Convert live ASR/NLU sample JSONL into reviewable eval candidates
  asr-nlu-candidate-eval Evaluate parser accuracy on reviewable ASR/NLU eval candidates
  release-gate        Job-showcase core 5-command gate with logs/acceptance_report.json
  demo-gate           Pre-demo automatic evidence gate with logs/demo_acceptance_report.json
  demo-evidence-checklist Summarize automatic/live/visual demo evidence into JSON/Markdown
  wsl-microphone-preflight PulseAudio/WSLg microphone capture check before live demos
  gazebo              Typed Action physical motion verification
  cpp-action-client   C++ rclcpp_action demo client sends typed command to simulation server
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
  continuous-live-check {offline|online}  Observe a running live microphone demo and score evidence
  continuous-nav2-live-check {offline|online}  Score a running live Nav2 microphone demo
  continuous-live-report REPORT_FILE  Re-score a saved continuous live-check report
  continuous-nav2-live-report REPORT_FILE  Re-score a saved Nav2 live-check report
EOF
}

if [[ "$LEVEL" == "help" || "$LEVEL" == "--help" || "$LEVEL" == "-h" ]]; then
  usage
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

run_base() {
  bash scripts/run_core_tests.sh
  colcon build --symlink-install --allow-overriding \
    embodied_agent_interfaces embodied_agent_cpp embodied_online_agent \
    embodied_offline_agent embodied_simulation
  colcon test --packages-select \
    embodied_agent_interfaces embodied_agent_cpp embodied_online_agent \
    embodied_offline_agent embodied_simulation \
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
  pytest -q tests/integration/test_voice_provider_preflight.py
  bash scripts/smoke_test_lifecycle.sh
  bash scripts/smoke_test_typed_action.sh
  bash scripts/smoke_test_typed_action_server.sh
  bash scripts/smoke_test_typed_action_pipeline.sh
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
  python3 scripts/sherpa_asr_smoke.py
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
  preflight) check_offline_runtime ;;
  mock) run_base ;;
  online) run_online ;;
  offline) run_offline ;;
  offline-runtime-versions) python3 scripts/offline_runtime_versions.py --check ;;
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
      ${OFFLINE_EVIDENCE_REQUIRE_ASR_TTS:+--require-asr-tts}
    ;;
  offline-latency) check_llama_cpp_runtime; bash scripts/smoke_test_offline_latency.sh ;;
  llama-cpp-preflight) check_llama_cpp_runtime; bash scripts/smoke_test_llama_cpp.sh preflight ;;
  llama-cpp-smoke) check_llama_cpp_runtime; bash scripts/smoke_test_llama_cpp.sh smoke ;;
  llama-decode-benchmark)
    check_llama_cpp_runtime
    require_file third_party/llama.cpp/build/bin/llama-bench
    python3 scripts/benchmark_llama_decode_speed.py \
      --minimum-decode-tokens-per-s "${LLAMA_DECODE_MIN_TOKENS_PER_S:-0}" \
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
  vad-sidecar) bash scripts/smoke_test_silero_vad_sidecar.sh ;;
  webrtc-vad-sidecar) bash scripts/smoke_test_webrtc_vad_sidecar.sh ;;
  kws-sidecar) bash scripts/smoke_test_keyword_wake_sidecar.sh ;;
  sherpa-kws-sidecar) bash scripts/smoke_test_sherpa_kws_sidecar.sh ;;
  openwakeword-sidecar) bash scripts/smoke_test_openwakeword_sidecar.sh ;;
  livekit-sidecar) bash scripts/smoke_test_livekit_wakeword_sidecar.sh ;;
  kws-calibration) bash scripts/smoke_test_kws_score_calibration.sh ;;
  voice-readiness) bash scripts/smoke_test_voice_readiness.sh ;;
  provider-preflight)
    pytest -q tests/integration/test_voice_provider_preflight.py
    python3 scripts/voice_provider_preflight.py \
      --mode "${PROVIDER_PREFLIGHT_MODE:-offline}" \
      --vad-provider "${VAD_PROVIDER:-auto}" \
      --kws-provider "${KWS_PROVIDER:-none}" \
      --silero-model-path "${SILERO_VAD_MODEL_PATH:-}" \
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
      --silero-model-path "${SILERO_VAD_MODEL_PATH:-}" \
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
  release-gate) python3 scripts/showcase_release_gate.py ;;
  demo-gate) python3 scripts/showcase_release_gate.py --profile demo ;;
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
  gazebo-voice) check_offline_runtime; USE_TYPED_ACTIONS=true bash scripts/smoke_test_gazebo_voice.sh ;;
  gazebo-voice-online) bash scripts/smoke_test_gazebo_voice_online.sh ;;
  microphone-offline) bash scripts/accept_voice_simulation_microphone.sh offline ;;
  microphone-online) bash scripts/accept_voice_simulation_microphone.sh online ;;
  continuous-offline) bash scripts/continuous_voice_control.sh offline ;;
  continuous-online) bash scripts/continuous_voice_control.sh online ;;
  continuous-nav2-offline) bash scripts/continuous_nav2_voice_control.sh offline ;;
  continuous-nav2-online) bash scripts/continuous_nav2_voice_control.sh online ;;
  continuous-nav2-evidence)
    CHECK_MODE="${2:-offline}"
    if [[ "$CHECK_MODE" != "offline" && "$CHECK_MODE" != "online" ]]; then
      echo "Usage: $0 continuous-nav2-evidence {offline|online}" >&2
      exit 2
    fi
    bash scripts/continuous_nav2_voice_evidence.sh "$CHECK_MODE"
    ;;
  continuous-live-check)
    CHECK_MODE="${2:-offline}"
    if [[ "$CHECK_MODE" != "offline" && "$CHECK_MODE" != "online" ]]; then
      echo "Usage: $0 continuous-live-check {offline|online}" >&2
      exit 2
    fi
    echo "continuous-live-check=$CHECK_MODE：请先在另一个终端启动 acceptance_test.sh continuous-$CHECK_MODE"
    LIVE_CHECK_ARGS=(--duration "${CONTINUOUS_LIVE_CHECK_DURATION:-180}")
    if [[ -n "${CONTINUOUS_LIVE_CHECK_REPORT:-}" ]]; then
      LIVE_CHECK_ARGS+=(--output "$CONTINUOUS_LIVE_CHECK_REPORT")
    fi
    python3 scripts/continuous_live_check.py "${LIVE_CHECK_ARGS[@]}"
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
