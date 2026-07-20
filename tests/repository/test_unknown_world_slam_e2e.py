"""Unknown-world 重型验收入口的失败留档契约。"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from tools.acceptance.scenarios.unknown_world_slam_e2e import (
    _run_probe_and_verify,
)


ROOT = Path(__file__).resolve().parents[2]


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
    return {
        "schema_version": 4,
        "passed": True,
        "session_id": session_id,
        "checks": {"map": True},
        "map_quality": {
            "metrics": {
                "reachable_free_coverage_ratio": 0.95,
                "region_coverage_ratios": {"main": 0.91},
            }
        },
        "localization": {"metrics": {"position_error_p95_m": 0.12}},
        "sampled_navigation": {"metrics": {"goal_count": 3}},
    }


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
