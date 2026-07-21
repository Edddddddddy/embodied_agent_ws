"""SLAM/Nav2 运行时探针的所有权与 CLI 契约。"""

from __future__ import annotations

import ast
import json
from pathlib import Path

import pytest

from tools.acceptance.probes.slam_nav.cli import build_parser
from tools.acceptance.probes.slam_nav.artifacts import build_failure_report


ROOT = Path(__file__).resolve().parents[2]
PROBE_ROOT = ROOT / "tools" / "acceptance" / "probes" / "slam_nav"
INTEGRATION_TEST_ROOT = ROOT / "tests" / "integration" / "slam_nav"

MIGRATED_EXECUTABLE_PROBES = {
    "continuous_navigation_natural.py",
    "continuous_navigation_queue.py",
    "dynamic_obstacle_tracker_ros.py",
    "lidar_loop_runtime.py",
    "nav2_bridge_sequence.py",
    "nav2_turtlebot3_voice.py",
    "navigation_sequence.py",
    "predicted_dynamic_obstacle_navigation.py",
    "slam_localization_navigation.py",
    "slam_mapping_baseline.py",
}


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
        "localization_sampler.py",
        "motion_evidence.py",
        "sampled_goal_tracker.py",
        "session_actions.py",
        "session_observer.py",
        "dynamic_scenario.py",
        "live_voice_trigger.py",
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


def test_slam_nav_tests_and_runtime_probes_have_distinct_semantic_owners():
    """pytest 文件只表达断言；会启动 ROS graph 的程序属于 acceptance tools。"""

    test_modules = sorted(
        path
        for path in INTEGRATION_TEST_ROOT.glob("*.py")
        if path.name != "__init__.py"
    )
    assert test_modules
    for path in test_modules:
        source = path.read_text(encoding="utf-8")
        tree = ast.parse(source, filename=str(path))
        assert path.name.startswith("test_")
        assert "__main__" not in source
        assert any(
            isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
            and node.name.startswith("test_")
            for node in tree.body
        ), f"{path.name} 必须包含可由 pytest 收集的测试函数"

    probe_modules = {
        path.name for path in PROBE_ROOT.glob("*.py") if path.name != "__init__.py"
    }
    assert MIGRATED_EXECUTABLE_PROBES <= probe_modules
    for name in MIGRATED_EXECUTABLE_PROBES:
        assert not name.startswith("test_")
        source = (PROBE_ROOT / name).read_text(encoding="utf-8")
        assert 'if __name__ == "__main__"' in source
        assert not (INTEGRATION_TEST_ROOT / f"test_{name}").exists()


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
    assert not any(
        name.endswith("session_observer")
        for module in (
            "localization_sampler",
            "motion_evidence",
            "sampled_goal_tracker",
            "session_actions",
        )
        for name in imports[module]
    )


def test_dynamic_scenario_transaction_policy_remains_ros_free():
    imports = _imports(
        ROOT / "tools/acceptance/dynamic_scenario_transaction.py"
    )

    assert not any(
        name.startswith(
            (
                "rclpy",
                "action_msgs",
                "geometry_msgs",
                "nav2_msgs",
                "nav_msgs",
            )
        )
        for name in imports
    )


def test_probe_modules_remain_bounded_deep_modules():
    limits = {
        # 真人语音只在此文件增加 ROS topic wiring；窗口关联与 PASS/FAIL
        # 已下沉到 voice_trigger_evidence 深模块，Adapter 仍设明确上限防止膨胀。
        "session_observer.py": 590,
        "sampled_goal_tracker.py": 320,
        "localization_sampler.py": 250,
        "motion_evidence.py": 140,
        "session_actions.py": 160,
        # 动态 costmap 的时序锁存独立于场景事务，防止报告证据逻辑重新
        # 膨胀回 dynamic_scenario.py。
        "dynamic_cost_evidence.py": 140,
        "dynamic_scenario.py": 500,
        "mapping_completion_adapter.py": 120,
        "live_voice_trigger.py": 120,
        "session_orchestrator.py": 610,
    }

    for filename, maximum in limits.items():
        lines = (PROBE_ROOT / filename).read_text(encoding="utf-8").splitlines()
        assert len(lines) <= maximum, f"{filename} grew to {len(lines)} lines"


def test_slam_stage_builds_the_agent_core_used_by_runtime_probes():
    handler = (
        ROOT / "tools/acceptance/handlers/slam_nav.sh"
    ).read_text(encoding="utf-8")

    # 干净 worktree 没有旧 overlay 可借用；session_observer 的 QoS/transport
    # 来自 embodied_agent_core，stage 必须把这个运行依赖一并建好。
    assert handler.count(
        "--packages-up-to embodied_agent_core embodied_slam_tools"
    ) == 2


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
    assert args.automatic_trigger_source == "synthetic"
    assert args.agent_mode == "offline"
    assert args.voice_trigger_timeout == 90.0
    assert not args.automatic_mission
    assert not args.cancel_automatic_mission


def test_cli_accepts_an_explicit_live_voice_trigger_contract(tmp_path):
    args = build_parser().parse_args(
        [
            "--output",
            str(tmp_path / "voice-report.json"),
            "--automatic-mission",
            "--automatic-trigger-source",
            "live_voice",
            "--agent-mode",
            "online",
            "--voice-trigger-timeout",
            "45.5",
        ]
    )

    assert args.automatic_trigger_source == "live_voice"
    assert args.agent_mode == "online"
    assert args.voice_trigger_timeout == 45.5


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
        mapping_completion_evidence={"valid": False},
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


def test_unknown_world_failure_report_is_explicitly_incomplete_schema_v4():
    report = build_failure_report(
        session_id="session-unknown",
        session_start_ns=456,
        evidence_kind="unknown_world_slam_nav_dynamic_replan",
        error="frontier recovery exhausted",
        state_sequence=[1, 2, 10],
        last_state_detail="frontier exploration",
        map_stats={"known_cells": 1000},
        frontier_goal_count=4,
        mapping_path_m=2.0,
        final_cmd_vel={"linear_x": 0.0, "angular_z": 0.0},
        schema_version=4,
        mission_outcome=3,
        mission_message="exploration failed",
    )

    assert report["schema_version"] == 4
    assert report["passed"] is False
    assert report["report_state"] == "incomplete"
    assert report["checks"] == {"evidence_complete": False}
    assert report["mission_outcome"] == 3
    assert report["mapping_completion"] is None


def test_unknown_world_failure_report_preserves_final_frontier_action_ledger():
    telemetry = {
        "valid": True,
        "status": "recovery_required:blacklisted_frontiers",
        "detected_frontier_count": 8,
        "available_frontier_count": 0,
        "blacklisted_frontier_count": 2,
        "active_goal_count": 0,
        "active_goal_id": "",
        "accepted_goal_count": 32,
        "succeeded_goal_count": 18,
        "aborted_goal_count": 12,
        "canceled_goal_count": 2,
        "rejected_goal_count": 0,
        "last_goal_terminal": "aborted",
        "provider_completion_reason": "blacklisted_frontiers",
        "mission_completion_reason": "recovery_failed",
    }

    report = build_failure_report(
        session_id="session-frontier-failure",
        session_start_ns=789,
        evidence_kind="unknown_world_slam_nav_dynamic_replan",
        error="frontier recovery stayed below material map gain",
        state_sequence=[1, 2, 10, 2],
        last_state_detail="automatic mission failed",
        map_stats={"known_cells": 26030},
        frontier_goal_count=32,
        mapping_path_m=111.09,
        final_cmd_vel={"linear_x": 0.0, "angular_z": 0.0},
        schema_version=4,
        mission_outcome=3,
        mission_message="recovery failed",
        frontier_telemetry=telemetry,
    )

    # 中途失败同样必须留下最终 typed frontier/Action 账本，不能要求操作者
    # 再从 runtime.log 的自然语言中反推 accepted/aborted 等关键计数。
    assert report["frontier"] == {"telemetry": telemetry}


def test_unknown_world_failure_report_preserves_typed_mapping_completion():
    mapping_completion = {
        "valid": False,
        "mode": "bounded_saturation",
        "return_home": {
            "passed": False,
            "checks": {
                "typed_action_succeeded": True,
                "final_cmd_vel_zero": True,
                "final_cmd_vel_fresh": False,
            },
            "action": {
                "command_id": "slam-return-home-1",
                "finished_at_ns": 5_000_000_000,
            },
        },
    }

    report = build_failure_report(
        session_id="session-return-failure",
        session_start_ns=789,
        evidence_kind="unknown_world_slam_nav_dynamic_replan",
        error="return evidence stale",
        state_sequence=[1, 2, 10, 3, 2],
        last_state_detail="automatic mission failed",
        map_stats={"known_cells": 26030},
        frontier_goal_count=32,
        mapping_path_m=111.09,
        final_cmd_vel={"linear_x": 0.0, "angular_z": 0.0},
        schema_version=4,
        mission_outcome=3,
        mission_message="return evidence stale",
        mapping_completion_evidence=mapping_completion,
    )

    assert report["mapping_completion"] == mapping_completion
    assert report["passed"] is False
    assert report["report_state"] == "incomplete"
    assert report["checks"] == {"evidence_complete": False}
    assert json.loads(json.dumps(report))["mapping_completion"] == mapping_completion


def test_session_orchestrator_wires_final_frontier_ledger_into_failure_report():
    tree = ast.parse(
        (PROBE_ROOT / "session_orchestrator.py").read_text(encoding="utf-8")
    )
    calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "build_failure_report"
    ]

    assert len(calls) == 1
    keywords = {item.arg: item.value for item in calls[0].keywords if item.arg}
    assert ast.unparse(keywords["frontier_telemetry"]) == "node.frontier_evidence"
    assert (
        ast.unparse(keywords["mapping_completion_evidence"])
        == "node.mapping_completion_evidence"
    )
