"""真人语音触发 strict unknown-world 任务的联合证据契约。"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from embodied_slam_tools.voice_trigger_evidence import (  # noqa: E402
    VoiceTriggerEvidenceWindow,
)


def _passing_core_report() -> dict[str, object]:
    checks = {
        "unknown_world_profile": True,
        "mission_sequence_present": True,
        "mission_completed": True,
        "mission_outcome_succeeded": True,
        "map_saved": True,
        "fresh_session_map": True,
        "map_quality": True,
        "frontier_complete": True,
        "localization_quality": True,
        "sampled_navigation": True,
        "nav2_lifecycle_active": True,
        "dynamic_navigation": True,
        "cmd_vel_observed": True,
        "robot_motion_observed": True,
        "final_task_motion_observed": True,
        "final_cmd_vel_fresh": True,
        "final_cmd_vel_zero": True,
    }
    return {
        "schema_version": 4,
        "evidence_kind": "unknown_world_slam_nav_dynamic_replan",
        "passed": True,
        "session_id": "unit-session",
        "session_start_ns": 1_000,
        "mission_sequence": 1,
        "checks": checks,
        "map_quality": {"passed": True},
        "frontier": {"passed": True},
        "localization": {"passed": True},
        "sampled_navigation": {"passed": True},
        "dynamic_navigation": {"passed": True},
    }


def _voice_window(*, omitted: str = "") -> VoiceTriggerEvidenceWindow:
    evidence = VoiceTriggerEvidenceWindow(window_started_at_ns=100)
    if omitted != "endpoint_start":
        evidence.record_endpoint(
            observed_at_ns=105,
            event="speech_started",
            provider="energy_vad",
        )
    if omitted != "audio":
        evidence.record_audio(
            observed_at_ns=110, speech=True, rms=0.08, peak=4_000
        )
    if omitted != "endpoint_end":
        evidence.record_endpoint(
            observed_at_ns=120,
            event="speech_ended",
            provider="energy_vad",
        )
    if omitted != "asr":
        evidence.record_asr_final(
            observed_at_ns=124, text="小智，开始自动建图"
        )
    if omitted != "wake":
        evidence.record_wake(
            observed_at_ns=125,
            kind=1,
            provider="text",
            transcript="小智，开始自动建图",
            command_known=True,
            command="开始自动建图",
        )
    return evidence


def test_real_voice_trigger_and_passing_core_build_joint_pass_envelope():
    evidence = VoiceTriggerEvidenceWindow(window_started_at_ns=100)
    evidence.record_audio(
        observed_at_ns=110, speech=True, rms=0.08, peak=4_000
    )
    evidence.record_endpoint(
        observed_at_ns=105, event="speech_started", provider="energy_vad"
    )
    evidence.record_endpoint(
        observed_at_ns=120, event="speech_ended", provider="energy_vad"
    )
    evidence.record_wake(
        observed_at_ns=125,
        kind=1,
        provider="keyword",
        transcript="小智，开始自动建图",
        command_known=True,
        command="开始自动建图",
    )
    evidence.record_asr_final(
        observed_at_ns=124, text="小智，开始自动建图"
    )

    envelope = evidence.build_envelope(
        _passing_core_report(), agent_mode="offline"
    )

    assert envelope["schema_version"] == 1
    assert envelope["evidence_kind"] == "voice_unknown_world_slam_nav_e2e"
    assert envelope["passed"] is True
    assert envelope["session_id"] == "unit-session"
    assert envelope["agent_mode"] == "offline"
    assert envelope["trigger_source"] == "live_voice"
    assert all(envelope["checks"].values())
    assert envelope["core_report"]["schema_version"] == 4


def test_later_real_command_is_selected_after_an_unusable_earlier_utterance():
    evidence = VoiceTriggerEvidenceWindow(window_started_at_ns=100)
    evidence.record_endpoint(
        observed_at_ns=110, event="speech_started", provider="energy_vad"
    )
    evidence.record_endpoint(
        observed_at_ns=120, event="speech_ended", provider="energy_vad"
    )
    evidence.record_asr_final(observed_at_ns=125, text="嗯")

    evidence.record_endpoint(
        observed_at_ns=200, event="speech_started", provider="energy_vad"
    )
    evidence.record_audio(
        observed_at_ns=210, speech=True, rms=0.07, peak=3_200
    )
    evidence.record_endpoint(
        observed_at_ns=230, event="speech_ended", provider="energy_vad"
    )
    evidence.record_asr_final(
        observed_at_ns=240, text="小智，开始自动巡检建图"
    )
    evidence.record_wake(
        observed_at_ns=245,
        kind=1,
        provider="text",
        transcript="小智，开始自动巡检建图",
        command_known=True,
        command="开始自动巡检建图",
    )

    envelope = evidence.build_envelope(
        _passing_core_report(), agent_mode="offline"
    )

    assert envelope["passed"] is True
    assert envelope["voice_window"]["matched_endpoint"] == {
        "speech_started_at_ns": 200,
        "speech_ended_at_ns": 230,
    }


@pytest.mark.parametrize("wake_kind", (1, 2))
def test_wake_and_continue_events_are_both_accepted_session_evidence(wake_kind):
    evidence = VoiceTriggerEvidenceWindow(window_started_at_ns=100)
    evidence.record_endpoint(
        observed_at_ns=105, event="speech_started", provider="energy_vad"
    )
    evidence.record_audio(
        observed_at_ns=110, speech=True, rms=0.08, peak=4_000
    )
    evidence.record_endpoint(
        observed_at_ns=120, event="speech_ended", provider="energy_vad"
    )
    evidence.record_asr_final(
        observed_at_ns=124, text="开始自动建图"
    )
    evidence.record_wake(
        observed_at_ns=125,
        kind=wake_kind,
        provider="text",
        transcript="开始自动建图",
        command_known=True,
        command="开始自动建图",
    )

    envelope = evidence.build_envelope(
        _passing_core_report(), agent_mode="offline"
    )

    assert envelope["checks"]["wake_accepted"] is True
    assert envelope["passed"] is True


def test_events_before_window_cannot_be_reused_as_current_voice_evidence():
    evidence = VoiceTriggerEvidenceWindow(window_started_at_ns=100)

    assert evidence.record_endpoint(
        observed_at_ns=90, event="speech_started", provider="energy_vad"
    ) is False
    assert evidence.record_audio(
        observed_at_ns=92, speech=True, rms=0.08, peak=4_000
    ) is False
    assert evidence.record_endpoint(
        observed_at_ns=95, event="speech_ended", provider="energy_vad"
    ) is False
    assert evidence.record_asr_final(
        observed_at_ns=96, text="小智，开始自动建图"
    ) is False
    assert evidence.record_wake(
        observed_at_ns=97,
        kind=1,
        provider="text",
        transcript="小智，开始自动建图",
        command_known=True,
        command="开始自动建图",
    ) is False

    envelope = evidence.build_envelope(
        _passing_core_report(), agent_mode="offline"
    )

    assert envelope["passed"] is False
    assert envelope["voice_window"]["audio_metrics"] == []
    assert envelope["voice_window"]["endpoint_events"] == []
    assert envelope["voice_window"]["wake_events"] == []
    assert envelope["voice_window"]["asr_finals"] == []


@pytest.mark.parametrize(
    ("omitted", "failed_check"),
    (
        ("audio", "real_audio_observed"),
        ("endpoint_start", "endpoint_observed"),
        ("endpoint_end", "endpoint_observed"),
        ("wake", "wake_accepted"),
        ("asr", "automatic_mission_asr_final"),
    ),
)
def test_joint_envelope_fails_closed_when_voice_fact_is_missing(
    omitted, failed_check
):
    envelope = _voice_window(omitted=omitted).build_envelope(
        _passing_core_report(), agent_mode="offline"
    )

    assert envelope["checks"][failed_check] is False
    assert envelope["passed"] is False


@pytest.mark.parametrize(
    ("command_known", "command"),
    (
        (False, "开始自动建图"),
        (True, ""),
        (True, "向前走一秒"),
    ),
)
def test_raw_asr_cannot_substitute_for_an_authorized_wake_command(
    command_known, command
):
    evidence = _voice_window(omitted="wake")
    evidence.record_wake(
        observed_at_ns=125,
        kind=1,
        provider="text",
        transcript="小智，开始自动建图",
        command_known=command_known,
        command=command,
    )

    envelope = evidence.build_envelope(
        _passing_core_report(), agent_mode="offline"
    )

    assert envelope["checks"]["automatic_mission_asr_final"] is True
    assert envelope["checks"]["wake_accepted"] is False
    assert envelope["voice_window"]["matched_wake_event"] is None
    assert envelope["passed"] is False


@pytest.mark.parametrize(
    ("field", "tampered_value"),
    (
        ("schema_version", 3),
        ("schema_version", 4.0),
        ("evidence_kind", "legacy_unknown_world_report"),
        ("passed", False),
        ("session_id", ""),
        ("session_start_ns", 0),
        ("mission_sequence", 0),
    ),
)
def test_core_schema_v4_top_level_tampering_is_rejected(
    field, tampered_value
):
    core_report = _passing_core_report()
    core_report[field] = tampered_value

    envelope = _voice_window().build_envelope(
        core_report, agent_mode="offline"
    )

    assert envelope["checks"]["strict_core_schema_v4_passed"] is False
    assert envelope["passed"] is False


def test_core_passed_flag_cannot_hide_failed_check_or_nested_section():
    failed_check_report = _passing_core_report()
    failed_check_report["checks"]["map_quality"] = False
    failed_section_report = _passing_core_report()
    failed_section_report["frontier"]["passed"] = False

    check_envelope = _voice_window().build_envelope(
        failed_check_report, agent_mode="offline"
    )
    section_envelope = _voice_window().build_envelope(
        failed_section_report, agent_mode="offline"
    )

    assert failed_check_report["passed"] is True
    assert check_envelope["passed"] is False
    assert section_envelope["passed"] is False
