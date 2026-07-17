#!/usr/bin/env bash
# Registered control acceptance handlers; sourced by the Python runner.

accept_core() {
  bash scripts/run_core_tests.sh
}

accept_architecture_facts() {
  python3 scripts/generate_architecture_facts.py --workspace "$WORKSPACE" --check
}

accept_agent_lifecycle() {
  bash scripts/smoke_test_agent_lifecycle.sh online; bash scripts/smoke_test_agent_lifecycle.sh offline
}

accept_preflight() {
  check_offline_runtime
}

accept_mock() {
  run_base
}

accept_demo() {
  bash scripts/smoke_test_demo_sequence.sh
}

accept_release_gate() {
  python3 scripts/showcase_release_gate.py --workspace "$WORKSPACE"
}

accept_robotics_gate() {
  python3 scripts/showcase_release_gate.py --workspace "$WORKSPACE" --profile robotics
}

accept_demo_gate() {
  python3 scripts/showcase_release_gate.py --workspace "$WORKSPACE" --profile demo
}

accept_demo_evidence_checklist() {
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
}

accept_wsl_microphone_preflight() {
  bash scripts/wsl_microphone_preflight.sh
}

accept_gazebo() {
  run_gazebo
}

accept_cpp_action_client() {
  bash scripts/smoke_test_cpp_action_client.sh
}

accept_cpp_action_scheduler() {
  bash scripts/smoke_test_cpp_action_scheduler.sh
}

accept_cpp_action_bridge_lifecycle() {
  bash scripts/smoke_test_typed_action_bridge_lifecycle.sh
}

accept_gazebo_voice() {
  check_offline_runtime; bash scripts/smoke_test_gazebo_voice.sh
}

accept_gazebo_voice_online() {
  bash scripts/smoke_test_gazebo_voice_online.sh
}

accept_runtime_evidence_summary() {
  python3 scripts/generate_runtime_evidence_summary.py
}

accept_all() {
  run_base; run_online; run_offline; bash scripts/smoke_test_demo_sequence.sh; run_gazebo; bash scripts/smoke_test_gazebo_voice.sh; bash scripts/smoke_test_gazebo_voice_online.sh
}
