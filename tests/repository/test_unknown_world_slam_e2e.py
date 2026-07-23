"""Unknown-world 重型验收入口的失败留档契约。"""

from __future__ import annotations

import json
import subprocess
from pathlib import Path

import pytest

from tools.acceptance.scenarios import voice_unknown_world_slam_e2e
from tools.acceptance.scenarios.unknown_world_slam_e2e import (
    UnknownWorldRunProfile,
    _build_orchestrator_command,
    _build_probe_command,
    _resolve_profile_runtime_environment,
    _run_probe_and_verify,
    _source_revision_environment,
    _verify_profile_report,
    verify_report,
)


ROOT = Path(__file__).resolve().parents[2]


def test_source_revision_environment_binds_clean_and_dirty_git_state(tmp_path):
    subprocess.run(["git", "init", "-q"], cwd=tmp_path, check=True)
    subprocess.run(
        ["git", "config", "user.email", "acceptance@example.invalid"],
        cwd=tmp_path,
        check=True,
    )
    subprocess.run(
        ["git", "config", "user.name", "Acceptance Test"],
        cwd=tmp_path,
        check=True,
    )
    tracked = tmp_path / "tracked.txt"
    tracked.write_text("baseline\n", encoding="utf-8")
    subprocess.run(["git", "add", "tracked.txt"], cwd=tmp_path, check=True)
    subprocess.run(["git", "commit", "-qm", "baseline"], cwd=tmp_path, check=True)

    clean = _source_revision_environment(tmp_path)
    assert len(clean["ACCEPTANCE_SOURCE_REVISION"]) == 40
    assert clean["ACCEPTANCE_SOURCE_DIRTY"] == "false"

    tracked.write_text("changed\n", encoding="utf-8")
    assert _source_revision_environment(tmp_path)[
        "ACCEPTANCE_SOURCE_DIRTY"
    ] == "true"


def test_source_revision_environment_marks_source_archive_unknown(tmp_path):
    assert _source_revision_environment(tmp_path) == {
        "ACCEPTANCE_SOURCE_REVISION": "unavailable",
        "ACCEPTANCE_SOURCE_DIRTY": "unknown",
    }


def test_live_voice_profile_enables_real_audio_without_startup_utterance_race():
    profile = UnknownWorldRunProfile.live_voice("offline")

    assert profile.agent_mode == "offline"
    assert profile.trigger_source == "live_voice"
    assert profile.artifact_directory_name == "voice_unknown_world_slam_nav"
    assert profile.report_filename == "voice_unknown_world_slam_e2e_report.json"
    assert profile.mapping_startup_timeout_s == 240.0
    assert profile.runtime_environment == {
        "NAV2_PROVIDER_MODE": "offline",
        "NAV2_MICROPHONE_ENABLED": "true",
        "NAV2_CAPTURE_ENABLED": "true",
        "WAKE_WORD_ENABLED": "true",
        "CONTINUOUS_PREFLIGHT_ENABLED": "true",
        "CONTINUOUS_MONITOR_ENABLED": "true",
        # readiness 会在 orchestrator 进入 MAPPING 前要求说话；联合探针必须在
        # MAPPING 后独占触发窗口，避免正确命令被 STARTING_MAPPING 拒绝。
        "CONTINUOUS_READINESS_ENABLED": "false",
        "CONTINUOUS_READINESS_REQUIRED": "false",
        "PULSE_CAPTURE_BRIDGE": "auto",
        "SPEAKER_ENABLED": "false",
        "SYSTEM_READINESS_TIMEOUT": "180",
    }


def test_synthetic_profile_preserves_existing_no_audio_gate():
    profile = UnknownWorldRunProfile.synthetic()

    assert profile.agent_mode == "offline"
    assert profile.trigger_source == "synthetic"
    assert profile.artifact_directory_name == "unknown_world_slam_nav"
    assert profile.report_filename == "unknown_world_slam_e2e_report.json"
    assert profile.mapping_startup_timeout_s == 150.0
    assert profile.runtime_environment["NAV2_PROVIDER_MODE"] == "mock"
    assert profile.runtime_environment["NAV2_MICROPHONE_ENABLED"] == "false"
    assert profile.runtime_environment["WAKE_WORD_ENABLED"] == "false"
    # 合成门禁不加载模型，正常约数秒就绪；Lifecycle 卡死必须尽快暴露，
    # 不能让用户面对一个不动的 Gazebo 界面等待两分钟。
    assert profile.runtime_environment["SYSTEM_READINESS_TIMEOUT"] == "45"


def test_live_voice_profile_rejects_unknown_agent_mode():
    with pytest.raises(ValueError, match="offline or online"):
        UnknownWorldRunProfile.live_voice("hybrid")


def test_live_runtime_resolves_wslg_pulse_socket_when_parent_env_is_missing(
    tmp_path,
):
    pulse_socket = tmp_path / "PulseServer"
    pulse_socket.touch()

    environment = _resolve_profile_runtime_environment(
        UnknownWorldRunProfile.live_voice("offline"),
        inherited_environment={},
        pulse_socket=pulse_socket,
    )

    assert environment["PULSE_SERVER"] == f"unix:{pulse_socket}"


def test_live_runtime_preserves_explicit_pulse_server(tmp_path):
    pulse_socket = tmp_path / "PulseServer"
    pulse_socket.touch()

    environment = _resolve_profile_runtime_environment(
        UnknownWorldRunProfile.live_voice("offline"),
        inherited_environment={"PULSE_SERVER": "tcp:10.0.0.2:4713"},
        pulse_socket=pulse_socket,
    )

    assert environment["PULSE_SERVER"] == "tcp:10.0.0.2:4713"


def test_live_runtime_records_resolved_asset_root_in_session_environment(tmp_path):
    runtime_root = tmp_path / "shared-runtime"
    runtime_root.mkdir()

    environment = _resolve_profile_runtime_environment(
        UnknownWorldRunProfile.live_voice("offline"),
        inherited_environment={"EMBODIED_RUNTIME_ROOT": str(runtime_root)},
        pulse_socket=tmp_path / "missing-pulse-socket",
    )

    assert environment["EMBODIED_RUNTIME_ROOT"] == str(runtime_root)


def test_live_profile_is_forwarded_to_orchestrator_and_probe_interfaces(tmp_path):
    profile = UnknownWorldRunProfile.live_voice("online")
    orchestrator = _build_orchestrator_command(
        profile=profile,
        workspace=tmp_path,
        map_prefix=tmp_path / "map",
        mission_plan=tmp_path / "mission.yaml",
    )
    probe = _build_probe_command(
        profile=profile,
        workspace=tmp_path,
        report_path=tmp_path / profile.report_filename,
        transition_timeout_s=10.0,
        gate_timeout_s=20.0,
        heartbeat_s=3.0,
        runtime_log=tmp_path / "runtime.log",
        session_id="session-live",
        session_started_ns=123,
        world_file=tmp_path / "world.sdf",
        mission_plan=tmp_path / "mission.yaml",
        dynamic_scenario=tmp_path / "dynamic.json",
        scene_spec=tmp_path / "scene.yaml",
        truth_map=tmp_path / "truth.yaml",
        voice_trigger_timeout_s=120.0,
    )

    assert "mode:=online" in orchestrator
    assert "command_input_source:=wake_event" in orchestrator
    assert probe[probe.index("--automatic-trigger-source") + 1] == "live_voice"
    assert probe[probe.index("--agent-mode") + 1] == "online"
    assert probe[probe.index("--voice-trigger-timeout") + 1] == "120.0"
    assert probe[probe.index("--gate-timeout-s") + 1] == "140.0"
    assert probe[probe.index("--evidence-kind") + 1] == (
        "voice_unknown_world_slam_nav_e2e"
    )


def test_synthetic_orchestrator_keeps_raw_asr_command_driver(tmp_path):
    orchestrator = _build_orchestrator_command(
        profile=UnknownWorldRunProfile.synthetic(),
        workspace=tmp_path,
        map_prefix=tmp_path / "map",
        mission_plan=tmp_path / "mission.yaml",
    )

    assert "command_input_source:=raw_asr" in orchestrator


def test_live_voice_wrapper_selects_requested_mode_and_shared_runner(monkeypatch):
    captured: list[UnknownWorldRunProfile] = []

    def fake_run(profile: UnknownWorldRunProfile) -> int:
        captured.append(profile)
        return 73

    monkeypatch.setattr(voice_unknown_world_slam_e2e, "run", fake_run)

    assert voice_unknown_world_slam_e2e.main(["online"]) == 73
    assert len(captured) == 1
    assert captured[0].agent_mode == "online"
    assert captured[0].trigger_source == "live_voice"


def test_real_orchestrator_spawn_is_audited_with_final_session_environment():
    source = (
        ROOT / "tools/acceptance/scenarios/unknown_world_slam_e2e.py"
    ).read_text(encoding="utf-8")
    audit_position = source.index("audit_unknown_world_policy_spawn(", 1)
    spawn_position = source.index(
        'session.spawn("orchestrator", orchestrator_command', audit_position
    )

    # 必须审计 AcceptanceSession 完成 unset/update 后的最终环境，并且先审计
    # 再 spawn；只测试另造的 argv/env 会留下生产链路可绕过的假安全缝隙。
    assert "environment=session.environment" in source[
        audit_position:spawn_position
    ]
    assert audit_position < spawn_position


class _ProbeSession:
    def __init__(
        self,
        *,
        probe_error: Exception | None = None,
        map_save_error: BaseException | None = None,
    ) -> None:
        self.probe_error = probe_error
        self.map_save_error = map_save_error
        self.calls: list[dict] = []

    def run(self, argv, *, timeout_s, check=True):
        call = {
            "argv": tuple(argv),
            "timeout_s": timeout_s,
            "check": check,
        }
        self.calls.append(call)
        if "map_saver_cli" in argv:
            if self.map_save_error is not None:
                raise self.map_save_error
            prefix = Path(argv[argv.index("-f") + 1])
            prefix.with_suffix(".yaml").write_text(
                "image: failed.pgm\n", encoding="utf-8"
            )
            prefix.with_suffix(".pgm").write_bytes(b"P5\n1 1\n255\n\x00")
            return 0
        if self.probe_error is not None:
            raise self.probe_error
        return 0


def _valid_report(session_id: str) -> dict:
    checks = {
        "unknown_world_profile": True,
        "mission_sequence_present": True,
        "mission_completed": True,
        "mission_outcome_succeeded": True,
        "map_saved": True,
        "fresh_session_map": True,
        "map_quality": True,
        "frontier_complete": True,
        "return_to_start": True,
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
        "session_id": session_id,
        "session_start_ns": 123,
        "mission_sequence": 1,
        "checks": checks,
        "map_quality": {
            "passed": True,
            "metrics": {
                "reachable_free_coverage_ratio": 0.95,
                "region_coverage_ratios": {"main": 0.91},
            }
        },
        "frontier": {"passed": True},
        "return_to_start": {"passed": True},
        "localization": {
            "passed": True,
            "metrics": {"position_error_p95_m": 0.12},
        },
        "sampled_navigation": {
            "passed": True,
            "metrics": {"goal_count": 3},
        },
        "dynamic_navigation": {"passed": True},
    }


def test_live_voice_envelope_must_wrap_a_valid_strict_core_report():
    profile = UnknownWorldRunProfile.live_voice("online")
    envelope = {
        "schema_version": 1,
        "evidence_kind": "voice_unknown_world_slam_nav_e2e",
        "session_id": "session-live",
        "agent_mode": "online",
        "trigger_source": "live_voice",
        "passed": True,
        "checks": {
            "real_audio_observed": True,
            "endpoint_observed": True,
            "wake_accepted": True,
            "automatic_mission_asr_final": True,
            "strict_core_schema_v4_passed": True,
        },
        "voice_window": {
            "matched_endpoint": {"speech_started_at_ns": 1, "speech_ended_at_ns": 2},
            "matched_automatic_mission_final": {
                "observed_at_ns": 3,
                "text": "小智，开始自动巡检建图",
            },
            "matched_wake_event": {
                "observed_at_ns": 4,
                "kind": 1,
                "command_known": True,
                "command": "开始自动巡检建图",
            },
        },
        "core_report": _valid_report("session-live"),
    }

    summary = _verify_profile_report(
        envelope,
        profile=profile,
        expected_session_id="session-live",
    )

    assert summary["navigation_goal_count"] == 3


def test_live_voice_envelope_cannot_claim_pass_with_wrong_evidence_kind():
    profile = UnknownWorldRunProfile.live_voice("offline")
    envelope = {
        "schema_version": 1,
        "evidence_kind": "unknown_world_slam_nav_dynamic_replan",
        "passed": True,
        "session_id": "session-live",
        "core_report": _valid_report("session-live"),
    }

    with pytest.raises(ValueError, match="live-voice evidence envelope"):
        _verify_profile_report(
            envelope,
            profile=profile,
            expected_session_id="session-live",
        )


def test_live_voice_envelope_cannot_hide_a_failed_voice_check():
    profile = UnknownWorldRunProfile.live_voice("offline")
    envelope = {
        "schema_version": 1,
        "evidence_kind": "voice_unknown_world_slam_nav_e2e",
        "session_id": "session-live",
        "agent_mode": "offline",
        "trigger_source": "live_voice",
        "passed": True,
        "checks": {
            "real_audio_observed": False,
            "endpoint_observed": True,
            "wake_accepted": True,
            "automatic_mission_asr_final": True,
            "strict_core_schema_v4_passed": True,
        },
        "voice_window": {
            "matched_endpoint": None,
            "matched_automatic_mission_final": {"text": "开始自动建图"},
        },
        "core_report": _valid_report("session-live"),
    }

    with pytest.raises(ValueError, match="voice checks"):
        _verify_profile_report(
            envelope,
            profile=profile,
            expected_session_id="session-live",
        )


def test_live_voice_envelope_must_match_requested_agent_and_session():
    profile = UnknownWorldRunProfile.live_voice("online")
    envelope = {
        "schema_version": 1,
        "evidence_kind": "voice_unknown_world_slam_nav_e2e",
        "session_id": "another-session",
        "agent_mode": "offline",
        "trigger_source": "live_voice",
        "passed": True,
        "checks": {name: True for name in (
            "real_audio_observed",
            "endpoint_observed",
            "wake_accepted",
            "automatic_mission_asr_final",
            "strict_core_schema_v4_passed",
        )},
        "voice_window": {
            "matched_endpoint": {"speech_started_at_ns": 1, "speech_ended_at_ns": 2},
            "matched_automatic_mission_final": {"text": "开始自动建图"},
        },
        "core_report": _valid_report("session-live"),
    }

    with pytest.raises(ValueError, match="metadata"):
        _verify_profile_report(
            envelope,
            profile=profile,
            expected_session_id="session-live",
        )


def test_strict_core_report_requires_current_evidence_kind_and_all_checks():
    report = _valid_report("session-core")
    report["evidence_kind"] = "legacy_unknown_world_report"

    with pytest.raises(ValueError, match="evidence kind"):
        verify_report(report, expected_session_id="session-core")

    report = _valid_report("session-core")
    del report["checks"]["dynamic_navigation"]
    with pytest.raises(ValueError, match="required checks"):
        verify_report(report, expected_session_id="session-core")


def test_probe_failure_saves_diagnostic_map_before_reraising_same_error(tmp_path):
    original = RuntimeError("probe failed")
    session = _ProbeSession(probe_error=original)
    report_path = tmp_path / "report.json"
    failed_prefix = tmp_path / "failed_exploration_map"

    with pytest.raises(RuntimeError) as caught:
        _run_probe_and_verify(
            session,
            probe_command=["python3", "probe.py"],
            report_path=report_path,
            expected_session_id="session-1",
            gate_timeout_s=90.0,
            failed_map_prefix=failed_prefix,
        )

    assert caught.value is original
    assert failed_prefix.with_suffix(".yaml").is_file()
    assert failed_prefix.with_suffix(".pgm").is_file()
    assert session.calls[-1]["check"] is False
    assert session.calls[-1]["argv"][4:6] == ("-f", str(failed_prefix))
    assert not (tmp_path / "unknown_world_map.yaml").exists()


def test_report_rejection_also_saves_diagnostic_map(tmp_path):
    session = _ProbeSession()
    report_path = tmp_path / "report.json"
    report = _valid_report("session-1")
    report["passed"] = False
    report_path.write_text(json.dumps(report), encoding="utf-8")
    failed_prefix = tmp_path / "failed_exploration_map"

    with pytest.raises(ValueError, match="did not pass"):
        _run_probe_and_verify(
            session,
            probe_command=["python3", "probe.py"],
            report_path=report_path,
            expected_session_id="session-1",
            gate_timeout_s=90.0,
            failed_map_prefix=failed_prefix,
        )

    assert failed_prefix.with_suffix(".yaml").is_file()
    assert failed_prefix.with_suffix(".pgm").is_file()


def test_diagnostic_map_failure_does_not_mask_probe_error(tmp_path):
    original = RuntimeError("probe failed")
    session = _ProbeSession(
        probe_error=original,
        map_save_error=KeyboardInterrupt(),
    )

    with pytest.raises(RuntimeError) as caught:
        _run_probe_and_verify(
            session,
            probe_command=["python3", "probe.py"],
            report_path=tmp_path / "report.json",
            expected_session_id="session-1",
            gate_timeout_s=90.0,
            failed_map_prefix=tmp_path / "failed_exploration_map",
        )

    assert caught.value is original
