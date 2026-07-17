"""SLAM/Nav2 运行时探针的所有权与 CLI 契约。"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

from tools.acceptance.probes.slam_nav.cli import build_parser
from tools.acceptance.probes.slam_nav.artifacts import build_failure_report


ROOT = Path(__file__).resolve().parents[2]
PROBE_ROOT = ROOT / "tools" / "acceptance" / "probes" / "slam_nav"


def _imports(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    names: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            names.add(node.module)
    return names


def test_ros_executables_are_owned_by_tools_not_pytest_collection():
    expected = {
        "cli.py",
        "artifacts.py",
        "session_observer.py",
        "dynamic_scenario.py",
        "session_orchestrator.py",
        "dynamic_navigation.py",
    }

    assert expected <= {path.name for path in PROBE_ROOT.glob("*.py")}
    assert not (
        ROOT
        / "tests/integration/slam_nav/test_voice_slam_session_orchestrator.py"
    ).exists()
    assert not (
        ROOT / "tests/integration/slam_nav/test_showcase_dynamic_navigation.py"
    ).exists()


def test_probe_dependency_direction_has_no_test_or_cli_back_edges():
    imports = {
        path.stem: _imports(path)
        for path in PROBE_ROOT.glob("*.py")
        if path.name != "__init__.py"
    }

    assert all(
        not name.startswith("tests.")
        for module_imports in imports.values()
        for name in module_imports
    )
    assert not any(
        name.endswith(("dynamic_scenario", "session_orchestrator"))
        for name in imports["session_observer"]
    )
    assert not any(
        name.endswith(("session_orchestrator", ".cli"))
        for name in imports["dynamic_scenario"]
    )


def test_probe_modules_remain_bounded_deep_modules():
    limits = {
        "session_observer.py": 450,
        "dynamic_scenario.py": 500,
        "session_orchestrator.py": 600,
    }

    for filename, maximum in limits.items():
        lines = (PROBE_ROOT / filename).read_text(encoding="utf-8").splitlines()
        assert len(lines) <= maximum, f"{filename} grew to {len(lines)} lines"


def test_cli_contract_is_ros_free_and_stable(tmp_path):
    parser = build_parser()
    with pytest.raises(SystemExit):
        parser.parse_args([])

    output = tmp_path / "report.json"
    args = parser.parse_args(["--output", str(output)])

    assert args.output == output
    assert args.transition_timeout == 10.0
    assert args.evidence_kind == "dry_run_process_adapter"
    assert args.session_id == ""
    assert args.session_start_ns == 0
    assert args.dynamic_navigation_timeout == 180.0
    assert args.gate_timeout_s == 900.0
    assert args.progress_heartbeat_s == 15.0
    assert not args.automatic_mission
    assert not args.cancel_automatic_mission


def test_failure_report_keeps_the_schema_v3_diagnostic_contract():
    report = build_failure_report(
        session_id="session-1",
        session_start_ns=123,
        evidence_kind="gazebo",
        error="planner timeout",
        state_sequence=[1, 2],
        last_state_detail="mapping",
        map_stats=None,
        frontier_goal_count=2,
        mapping_path_m=1.2349,
        final_cmd_vel={"linear_x": 0.0, "angular_z": 0.0},
    )

    assert set(report) == {
        "schema_version",
        "passed",
        "session_id",
        "session_start_ns",
        "evidence_kind",
        "error",
        "state_sequence",
        "last_state_detail",
        "map",
        "frontier_goal_count",
        "mapping_path_m",
        "final_cmd_vel",
    }
    assert report["schema_version"] == 3
    assert report["passed"] is False
    assert report["mapping_path_m"] == 1.235
