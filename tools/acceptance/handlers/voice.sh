#!/usr/bin/env bash
# Registered voice acceptance handlers; sourced by the Python runner.

accept_online() {
  run_online
}

accept_offline() {
  run_offline
}

accept_offline_runtime_versions() {
  python3 scripts/offline_runtime_versions.py --check
}

accept_offline_voice_e2e_report() {
  bash scripts/smoke_test_offline_voice_real.sh
}

accept_offline_showcase_report() {
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
  python3 tools/evaluation/generate_offline_showcase_report.py "${REPORT_ARGS[@]}"
}

accept_offline_evidence_audit() {
  if [[ ! -s "${OFFLINE_EVIDENCE_REPORT:-logs/offline_showcase_report.json}" ]]; then
    python3 tools/evaluation/generate_offline_showcase_report.py
  fi
  python3 tools/evaluation/audit_offline_showcase_evidence.py \
    --input "${OFFLINE_EVIDENCE_REPORT:-logs/offline_showcase_report.json}" \
    --output "${OFFLINE_EVIDENCE_AUDIT_OUTPUT:-logs/offline_evidence_audit.json}" \
    ${OFFLINE_EVIDENCE_REQUIRE_LATENCY:+--require-latency} \
    ${OFFLINE_EVIDENCE_REQUIRE_LLAMA_BENCH:+--require-llama-bench} \
    ${OFFLINE_EVIDENCE_REQUIRE_INSTRUCTION_FOLLOWING:+--require-instruction-following} \
    ${OFFLINE_EVIDENCE_REQUIRE_ASR_TTS:+--require-asr-tts} \
    ${OFFLINE_EVIDENCE_REQUIRE_VOICE_E2E:+--require-voice-e2e}
}

accept_offline_latency() {
  check_llama_cpp_runtime; bash scripts/smoke_test_offline_latency.sh
}

accept_llama_cpp_preflight() {
  check_llama_cpp_runtime; bash scripts/smoke_test_llama_cpp.sh preflight
}

accept_llama_cpp_smoke() {
  check_llama_cpp_runtime; bash scripts/smoke_test_llama_cpp.sh smoke
}

accept_summer_tts_preflight() {
  check_summer_tts_runtime; python3 scripts/summer_tts_smoke.py --preflight-only
}

accept_summer_tts_smoke() {
  check_summer_tts_runtime; python3 scripts/summer_tts_smoke.py
}

accept_summer_pseudo_tts() {
  check_summer_tts_runtime; python3 scripts/smoke_test_summer_pseudo_tts.py
}

accept_summer_tts_service() {
  bash scripts/smoke_test_summer_tts_service.sh
}

accept_summer_tts_cache_audit() {
  python3 tools/evaluation/audit_summer_tts_cache_evidence.py \
    --input "${SUMMER_TTS_CACHE_PROBE_REPORT:-logs/summer_tts_service_probe.json}" \
    --output "${SUMMER_TTS_CACHE_AUDIT_REPORT:-logs/summer_tts_cache_audit.json}" \
    --target-roundtrip-ms "${SUMMER_TTS_CACHE_TARGET_MS:-300}" \
    --min-pcm-bytes "${SUMMER_TTS_CACHE_MIN_PCM_BYTES:-1000}" \
    ${SUMMER_TTS_CACHE_ALLOW_MISS:+--allow-cache-miss}
}

accept_pseudo_tts() {
  python3 scripts/smoke_test_pseudo_streaming_tts.py
}

accept_sherpa_asr_preflight() {
  run_sherpa_asr_preflight
}

accept_sherpa_asr_smoke() {
  run_sherpa_asr_smoke
}

accept_offline_sherpa_typed() {
  check_offline_runtime; bash scripts/smoke_test_offline_sherpa_typed_simulation.sh
}

accept_continuous_mock() {
  bash scripts/smoke_test_continuous_voice.sh online; bash scripts/smoke_test_continuous_voice.sh offline
}

accept_continuous_soak() {
  bash scripts/smoke_test_continuous_voice_soak.sh online; bash scripts/smoke_test_continuous_voice_soak.sh offline
}

accept_continuous_endpoint() {
  bash scripts/smoke_test_continuous_endpoint_asr.sh online; bash scripts/smoke_test_continuous_endpoint_asr.sh offline
}

accept_continuous_multi_command() {
  bash scripts/smoke_test_continuous_multi_command.sh online; bash scripts/smoke_test_continuous_multi_command.sh offline
}

accept_continuous_queue_full() {
  bash scripts/smoke_test_continuous_queue_full.sh online; bash scripts/smoke_test_continuous_queue_full.sh offline
}

accept_continuous_ttl() {
  bash scripts/smoke_test_continuous_command_ttl.sh online; bash scripts/smoke_test_continuous_command_ttl.sh offline
}

accept_continuous_timeout() {
  bash scripts/smoke_test_continuous_session_timeout.sh online; bash scripts/smoke_test_continuous_session_timeout.sh offline
}

accept_continuous_kws_mock() {
  bash scripts/smoke_test_continuous_kws_sidecar.sh online; bash scripts/smoke_test_continuous_kws_sidecar.sh offline
}

accept_speaker_memory_mock() {
  bash scripts/smoke_test_speaker_memory.sh online; bash scripts/smoke_test_speaker_memory.sh offline
}

accept_speaker_enroll() {
  bash scripts/smoke_test_speaker_enrollment.sh
}

accept_speaker_runtime() {
  source "$WORKSPACE/scripts/activate.sh"; python3 scripts/probe_sherpa_speaker_runtime.py; bash scripts/smoke_test_sherpa_speaker_identity.sh
}

accept_vad_sidecar() {
  bash scripts/smoke_test_silero_vad_sidecar.sh
}

accept_silero_vad_runtime() {
  python3 scripts/silero_onnx_smoke.py \
    --model "${SILERO_VAD_MODEL_PATH:-models/silero_vad/silero_vad.onnx}" \
    --output "${SILERO_VAD_REPORT:-logs/silero_vad_runtime.json}"
  bash scripts/smoke_test_silero_vad_runtime.sh
}

accept_webrtc_vad_sidecar() {
  bash scripts/smoke_test_webrtc_vad_sidecar.sh
}

accept_kws_sidecar() {
  bash scripts/smoke_test_keyword_wake_sidecar.sh
}

accept_sherpa_kws_sidecar() {
  bash scripts/smoke_test_sherpa_kws_sidecar.sh
}

accept_openwakeword_sidecar() {
  bash scripts/smoke_test_openwakeword_sidecar.sh
}

accept_livekit_sidecar() {
  bash scripts/smoke_test_livekit_wakeword_sidecar.sh
}

accept_kws_calibration() {
  bash scripts/smoke_test_kws_score_calibration.sh
}

accept_voice_readiness() {
  bash scripts/smoke_test_voice_readiness.sh
}

accept_provider_preflight() {
  python3 -m pytest -q tests/integration/voice/test_voice_provider_preflight.py
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
}

accept_voice_stability_preflight() {
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
}

accept_voice_vad_runtime_dry_run() {
  bash scripts/setup_voice_vad_runtime.sh "${VOICE_VAD_PROFILE:-all}" --dry-run
}

accept_voice_kws_runtime_dry_run() {
  bash scripts/setup_voice_kws_runtime.sh "${VOICE_KWS_PROFILE:-all}" --dry-run
}

accept_voice_calibration_report() {
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
}

accept_asr_nlu_samples_to_eval() {
  ASR_NLU_EVAL_OUTPUT="${ASR_NLU_EVAL_OUTPUT:-logs/asr_nlu_eval_candidates.jsonl}"
  if [[ "${ASR_NLU_SAMPLES_SYNTHETIC:-true}" == "true" ]]; then
    python3 tools/evaluation/asr_nlu_samples_to_eval_candidates.py \
      --synthetic-demo \
      --output "$ASR_NLU_EVAL_OUTPUT"
  else
    python3 tools/evaluation/asr_nlu_samples_to_eval_candidates.py \
      --input "${ASR_NLU_SAMPLE_LOG:-logs/asr_nlu_samples.jsonl}" \
      --output "$ASR_NLU_EVAL_OUTPUT"
  fi
}

accept_asr_nlu_candidate_eval() {
  ASR_NLU_CANDIDATE_INPUT="${ASR_NLU_CANDIDATE_INPUT:-logs/asr_nlu_eval_candidates.synthetic.jsonl}"
  if [[ "${ASR_NLU_CANDIDATE_SYNTHETIC:-true}" == "true" ]]; then
    python3 tools/evaluation/asr_nlu_samples_to_eval_candidates.py \
      --synthetic-demo \
      --output "$ASR_NLU_CANDIDATE_INPUT"
  fi
  python3 tools/evaluation/evaluate_asr_nlu_eval_candidates.py \
    --input "$ASR_NLU_CANDIDATE_INPUT" \
    --output "${ASR_NLU_CANDIDATE_REPORT:-logs/asr_nlu_candidate_eval_report.json}" \
    --minimum "${ASR_NLU_CANDIDATE_MINIMUM:-1.0}"
}

accept_continuous_offline() {
  bash scripts/continuous_voice_control.sh offline
}

accept_continuous_online() {
  bash scripts/continuous_voice_control.sh online
}

accept_continuous_voice_evidence() {
  CHECK_MODE="${1:-offline}"
  if [[ "$CHECK_MODE" != "offline" && "$CHECK_MODE" != "online" ]]; then
    echo "Usage: ${ACCEPTANCE_PROGRAM:-acceptance_test.sh} continuous-voice-evidence {offline|online}" >&2
    exit 2
  fi
  bash scripts/continuous_voice_evidence.sh "$CHECK_MODE"
}

accept_continuous_live_check() {
  CHECK_MODE="${1:-offline}"
  if [[ "$CHECK_MODE" != "offline" && "$CHECK_MODE" != "online" ]]; then
    echo "Usage: ${ACCEPTANCE_PROGRAM:-acceptance_test.sh} continuous-live-check {offline|online}" >&2
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
}

accept_continuous_live_report() {
  REPORT_PATH="${1:-}"
  if [[ -z "$REPORT_PATH" ]]; then
    echo "Usage: ${ACCEPTANCE_PROGRAM:-acceptance_test.sh} continuous-live-report REPORT_FILE" >&2
    exit 2
  fi
  python3 scripts/continuous_live_check.py --input-report "$REPORT_PATH"
}
