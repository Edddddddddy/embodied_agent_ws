from __future__ import annotations

import json
import io
from pathlib import Path

import pytest

import tools.acceptance.scenarios.project_verification as verification_module
from tools.acceptance.scenarios.project_verification import (
    PROFILE_CHECK_NAMES,
    VerificationCheck,
    _execute_check,
    checks_for_profile,
    run,
)


LIVE_MICROPHONE_MODES = {
    "continuous-offline",
    "continuous-online",
    "voice-unknown-world-slam-e2e",
}


def test_all_profile_covers_each_key_layer_without_live_microphone():
    checks = checks_for_profile("all")
    modes = {check.mode for check in checks}

    assert LIVE_MICROPHONE_MODES.isdisjoint(modes)
    assert {
        "core",
        "voice-readiness",
        "continuous-endpoint",
        "continuous-multi-command",
        "offline-sherpa-typed",
        "cpp-action-client",
        "cpp-action-scheduler",
        "control-authority-stage",
        "gazebo",
        "unknown-world-slam-e2e",
    } == modes


def test_skip_local_models_only_removes_real_offline_check():
    full = checks_for_profile("voice")
    reduced = checks_for_profile("voice", skip_local_models=True)

    assert len(full) == len(reduced) + 1
    assert all(not check.requires_local_models for check in reduced)
    assert {check.name for check in full} - {check.name for check in reduced} == {
        "voice_offline_model"
    }


def test_successful_profile_writes_machine_readable_report(tmp_path: Path):
    report_path = tmp_path / "report.json"
    observed: list[str] = []

    def fake_execute(
        check: VerificationCheck,
        _workspace: Path,
        _tail_lines: int,
    ) -> dict:
        observed.append(check.name)
        return {
            "name": check.name,
            "mode": check.mode,
            "timeout_s": check.timeout_s,
            "evidence": check.evidence,
            "requires_local_models": check.requires_local_models,
            "command": ["acceptance_test.sh", check.mode],
            "passed": True,
            "returncode": 0,
            "duration_s": 0.01,
            "output_tail": ["PASS"],
            "error": None,
        }

    status = run(
        "control",
        workspace=tmp_path,
        report_path=report_path,
        execute_check=fake_execute,
    )
    report = json.loads(report_path.read_text(encoding="utf-8"))

    assert status == 0
    assert observed == list(PROFILE_CHECK_NAMES["control"])
    assert report["scenario"] == "project_key_feature_verification"
    assert report["live_microphone_used"] is False
    assert report["passed"] is True
    assert report["not_run"] == []


def test_failure_stops_before_later_ros_stages(tmp_path: Path):
    report_path = tmp_path / "report.json"

    def fake_execute(
        check: VerificationCheck,
        _workspace: Path,
        _tail_lines: int,
    ) -> dict:
        return {
            "name": check.name,
            "mode": check.mode,
            "timeout_s": check.timeout_s,
            "evidence": check.evidence,
            "requires_local_models": check.requires_local_models,
            "command": ["acceptance_test.sh", check.mode],
            "passed": False,
            "returncode": 1,
            "duration_s": 0.01,
            "output_tail": ["failed"],
            "output_streamed": True,
            "error": "fixture failure",
        }

    status = run(
        "all",
        workspace=tmp_path,
        report_path=report_path,
        execute_check=fake_execute,
    )
    report = json.loads(report_path.read_text(encoding="utf-8"))

    assert status == 1
    assert len(report["checks"]) == 1
    assert report["checks"][0]["name"] == "core"
    assert report["not_run"][0] == "voice_frontend_readiness"
    assert "slam_nav" in report["not_run"]


def test_real_executor_streams_output_and_keeps_bounded_tail(
    tmp_path: Path,
    capsys,
):
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    entry = scripts / "acceptance_test.sh"
    entry.write_text(
        "#!/usr/bin/env bash\nprintf 'line-1\\nline-2\\nline-3\\n'\n",
        encoding="utf-8",
    )
    entry.chmod(0o755)
    check = VerificationCheck("fixture", "fixture", 5.0, "stream fixture")

    result = _execute_check(check, tmp_path, tail_lines=2)

    assert result["passed"] is True
    assert result["output_streamed"] is True
    assert result["output_tail"] == ["line-2", "line-3"]
    assert "line-1" in capsys.readouterr().out


def test_real_executor_cleans_process_group_when_user_interrupts(
    tmp_path: Path,
    monkeypatch,
):
    terminated: list[object] = []

    class InterruptedProcess:
        pid = 12345
        stdout = io.StringIO("")

        def wait(self, timeout=None):
            raise KeyboardInterrupt

    process = InterruptedProcess()
    monkeypatch.setattr(
        verification_module.subprocess,
        "Popen",
        lambda *args, **kwargs: process,
    )
    monkeypatch.setattr(
        verification_module,
        "_terminate_process_group",
        lambda value: terminated.append(value),
    )
    check = VerificationCheck("fixture", "fixture", 5.0, "interrupt fixture")

    with pytest.raises(KeyboardInterrupt):
        _execute_check(check, tmp_path, tail_lines=2)

    assert terminated == [process]


def test_real_executor_times_out_and_reaps_process_group(tmp_path: Path):
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    entry = scripts / "acceptance_test.sh"
    entry.write_text("#!/usr/bin/env bash\nsleep 30\n", encoding="utf-8")
    entry.chmod(0o755)
    check = VerificationCheck("fixture", "fixture", 0.1, "timeout fixture")

    result = _execute_check(check, tmp_path, tail_lines=2)

    assert result["passed"] is False
    assert result["returncode"] == 124
    assert result["error"] == "timeout after 0s"
