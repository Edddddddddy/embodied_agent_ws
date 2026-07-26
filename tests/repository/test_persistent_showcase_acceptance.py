"""持久 Gazebo 建图→导航验收的纯契约测试。"""

from __future__ import annotations

import copy
import io
import os
from pathlib import Path

import pytest

from tools.acceptance.catalog import MODE_BY_NAME
from tools.acceptance.cli import main as acceptance_main
from tools.acceptance.probes.slam_nav.cli import build_parser
from tools.acceptance.probes.slam_nav.runtime_continuity_adapter import (
    RuntimeContinuityProbe,
)
from tools.acceptance.runtime_continuity import (
    ProcessCandidate,
    RuntimeRoleSelectionError,
    build_runtime_continuity_evidence,
    select_runtime_role_pids,
)
from tools.acceptance.runtime_identity import RuntimeCheckpoint
from tools.acceptance.showcase_evidence import (
    verify_persistent_runtime_report,
)
from tools.acceptance.scenarios import showcase_gazebo_e2e
from tools.acceptance.scenarios.unknown_world_run_profile import (
    UnknownWorldRunProfile,
)
from tools.acceptance.scenarios.unknown_world_slam_e2e import (
    _build_orchestrator_command,
    _build_probe_command,
    _start_persistent_authority,
)

ROOT = Path(__file__).resolve().parents[2]


def _candidate(
    pid: int,
    executable: str,
    argv: tuple[str, ...],
    **environment: str,
) -> ProcessCandidate:
    return ProcessCandidate(
        pid=pid,
        executable=executable,
        argv=argv,
        environment=environment,
    )


def _persistent_candidates() -> tuple[ProcessCandidate, ...]:
    return (
        _candidate(
            101,
            "/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher",
            ("robot_state_publisher", "--ros-args"),
            ROS_DOMAIN_ID="171",
        ),
        _candidate(
            102,
            "/usr/bin/ruby3.2",
            ("ruby", "/usr/bin/gz", "sim", "-r", "-s", "showcase.sdf"),
            GZ_PARTITION="showcase_partition_171",
        ),
        # GUI 与 server 使用相同 partition；连续性证据只能绑定带 -s 的 server。
        _candidate(
            104,
            "/usr/bin/ruby3.2",
            ("ruby", "/usr/bin/gz", "sim", "-g"),
            GZ_PARTITION="showcase_partition_171",
        ),
        _candidate(
            103,
            "/opt/ros/jazzy/lib/rviz2/rviz2",
            ("rviz2", "-d", "showcase.rviz"),
            ROS_DOMAIN_ID="171",
        ),
        _candidate(
            106,
            "/usr/bin/python3.12",
            (
                "/workspace/install/embodied_offline_agent/lib/"
                "embodied_offline_agent/offline_agent",
                "--ros-args",
                "-r",
                "__node:=offline_agent",
            ),
            ROS_DOMAIN_ID="171",
        ),
        _candidate(
            107,
            "/usr/bin/python3.12",
            (
                "/workspace/install/embodied_online_agent/lib/"
                "embodied_online_agent/online_agent",
                "--ros-args",
            ),
            ROS_DOMAIN_ID="172",
        ),
        # 其他验收会话即使进程名相同，也不能混入本次证据。
        _candidate(
            201,
            "/usr/bin/ruby3.2",
            ("ruby", "/usr/bin/gz", "sim", "-r", "other.sdf"),
            GZ_PARTITION="other_partition",
        ),
    )


def test_runtime_selector_uses_domain_partition_and_optional_rviz() -> None:
    assert select_runtime_role_pids(
        _persistent_candidates(),
        ros_domain_id="171",
        gz_partition="showcase_partition_171",
        require_rviz=False,
        agent_mode="offline",
    ) == {
        "offline_agent": 106,
        "gazebo_server": 102,
        "robot_state_publisher": 101,
    }
    assert select_runtime_role_pids(
        _persistent_candidates(),
        ros_domain_id="171",
        gz_partition="showcase_partition_171",
        require_rviz=True,
        agent_mode="offline",
    ) == {
        "offline_agent": 106,
        "gazebo_server": 102,
        "robot_state_publisher": 101,
        "rviz": 103,
    }


def test_runtime_selector_accepts_gz_ruby_single_argv_process_title() -> None:
    candidates = list(_persistent_candidates())
    candidates[1] = _candidate(
        102,
        "/usr/bin/ruby3.2",
        ("gz sim -r -s /tmp/showcase.sdf",),
        GZ_PARTITION="showcase_partition_171",
    )

    assert select_runtime_role_pids(
        candidates,
        ros_domain_id="171",
        gz_partition="showcase_partition_171",
        require_rviz=False,
        agent_mode="offline",
    )["gazebo_server"] == 102


@pytest.mark.parametrize("failure", ["missing", "ambiguous"])
def test_runtime_selector_rejects_missing_or_ambiguous_roles(failure: str) -> None:
    candidates = list(_persistent_candidates())
    if failure == "missing":
        candidates = [candidate for candidate in candidates if candidate.pid != 102]
    else:
        candidates.append(
            _candidate(
                105,
                "/usr/bin/ruby3.2",
                (
                    "ruby",
                    "/usr/bin/gz",
                    "sim",
                    "-r",
                    "-s",
                    "duplicate.sdf",
                ),
                GZ_PARTITION="showcase_partition_171",
            )
        )

    with pytest.raises(RuntimeRoleSelectionError, match="gazebo_server"):
        select_runtime_role_pids(
            candidates,
            ros_domain_id="171",
            gz_partition="showcase_partition_171",
            require_rviz=False,
            agent_mode="offline",
        )


def test_runtime_selector_binds_the_agent_selected_by_agent_mode() -> None:
    candidates = [
        (
            _candidate(
                107,
                "/usr/bin/python3.12",
                (
                    "/workspace/install/embodied_online_agent/lib/"
                    "embodied_online_agent/online_agent",
                    "--ros-args",
                ),
                ROS_DOMAIN_ID="171",
            )
            if candidate.pid == 107
            else candidate
        )
        for candidate in _persistent_candidates()
    ]

    selected = select_runtime_role_pids(
        candidates,
        ros_domain_id="171",
        gz_partition="showcase_partition_171",
        require_rviz=False,
        agent_mode="online",
    )

    assert selected["online_agent"] == 107
    assert "offline_agent" not in selected


def test_runtime_selector_accepts_python_shebang_agent_process() -> None:
    """Linux shebang 会把解释器放在 argv[0]、真实脚本放在 argv[1]。"""

    candidates = [
        (
            _candidate(
                106,
                "/usr/bin/python3.12",
                (
                    "/usr/bin/python3",
                    "/workspace/install/embodied_offline_agent/lib/"
                    "embodied_offline_agent/offline_agent",
                    "--ros-args",
                    "-r",
                    "__node:=offline_agent",
                ),
                ROS_DOMAIN_ID="171",
            )
            if candidate.pid == 106
            else candidate
        )
        for candidate in _persistent_candidates()
    ]

    assert select_runtime_role_pids(
        candidates,
        ros_domain_id="171",
        gz_partition="showcase_partition_171",
        require_rviz=False,
        agent_mode="offline",
    )["offline_agent"] == 106


def test_runtime_selector_does_not_mistake_launch_parent_for_agent() -> None:
    candidates = [
        candidate
        for candidate in _persistent_candidates()
        if candidate.pid != 106
    ]
    candidates.append(
        _candidate(
            108,
            "/usr/bin/python3.12",
            (
                "/opt/ros/jazzy/bin/ros2",
                "launch",
                "embodied_simulation",
                "persistent_voice_nav_base.launch.py",
                "mode:=offline_agent",
            ),
            ROS_DOMAIN_ID="171",
        )
    )

    with pytest.raises(RuntimeRoleSelectionError, match="offline_agent"):
        select_runtime_role_pids(
            candidates,
            ros_domain_id="171",
            gz_partition="showcase_partition_171",
            require_rviz=False,
            agent_mode="offline",
        )


def test_continuity_evidence_requires_identical_kernel_process_identity() -> None:
    mapping = RuntimeCheckpoint.capture(
        "mapping_ready",
        {
            "offline_agent": os.getpid(),
            "gazebo_server": os.getpid(),
            "robot_state_publisher": os.getpid(),
        },
        wall_ns=100,
    )
    navigation = RuntimeCheckpoint.capture(
        "navigation_ready",
        {
            "offline_agent": os.getpid(),
            "gazebo_server": os.getpid(),
            "robot_state_publisher": os.getpid(),
        },
        wall_ns=200,
    )

    evidence = build_runtime_continuity_evidence(
        mapping,
        navigation,
        required_roles=(
            "gazebo_server",
            "offline_agent",
            "robot_state_publisher",
        ),
    )

    assert evidence["passed"] is True
    assert evidence["schema_version"] == 1
    assert evidence["checks"] == {
        "checkpoint_schema_valid": True,
        "checkpoint_labels_valid": True,
        "checkpoint_time_ordered": True,
        "required_roles_present": True,
        "role_sets_exact": True,
        "gazebo_server_identity_schema_valid": True,
        "gazebo_server_unchanged": True,
        "offline_agent_identity_schema_valid": True,
        "offline_agent_unchanged": True,
        "robot_state_publisher_identity_schema_valid": True,
        "robot_state_publisher_unchanged": True,
    }
    for checkpoint in evidence["checkpoints"].values():
        assert all(
            "argv" not in identity
            for identity in checkpoint["roles"].values()
        )


def test_continuity_rejects_equal_checkpoints_with_wrong_labels() -> None:
    checkpoint = RuntimeCheckpoint.capture(
        "mapping_ready",
        {
            "gazebo_server": os.getpid(),
            "offline_agent": os.getpid(),
        },
        wall_ns=100,
    ).as_evidence()
    mapping = copy.deepcopy(checkpoint)
    navigation = copy.deepcopy(checkpoint)
    mapping["label"] = "unexpected_mapping_label"
    navigation["label"] = "unexpected_navigation_label"
    navigation["wall_ns"] = 200

    evidence = build_runtime_continuity_evidence(
        mapping,
        navigation,
        required_roles=("gazebo_server", "offline_agent"),
    )

    assert evidence["passed"] is False
    assert evidence["checks"]["checkpoint_labels_valid"] is False


def test_continuity_rejects_reversed_checkpoint_time_order() -> None:
    mapping = RuntimeCheckpoint.capture(
        "mapping_ready",
        {
            "gazebo_server": os.getpid(),
            "offline_agent": os.getpid(),
        },
        wall_ns=200,
    )
    navigation = RuntimeCheckpoint.capture(
        "navigation_ready",
        {
            "gazebo_server": os.getpid(),
            "offline_agent": os.getpid(),
        },
        wall_ns=100,
    )

    evidence = build_runtime_continuity_evidence(
        mapping,
        navigation,
        required_roles=("gazebo_server", "offline_agent"),
    )

    assert evidence["passed"] is False
    assert evidence["checks"]["checkpoint_time_ordered"] is False


def test_continuity_rejects_equal_but_schema_drifted_identities() -> None:
    mapping = RuntimeCheckpoint.capture(
        "mapping_ready",
        {
            "gazebo_server": os.getpid(),
            "offline_agent": os.getpid(),
        },
        wall_ns=100,
    ).as_evidence()
    navigation = RuntimeCheckpoint.capture(
        "navigation_ready",
        {
            "gazebo_server": os.getpid(),
            "offline_agent": os.getpid(),
        },
        wall_ns=200,
    ).as_evidence()
    for checkpoint in (mapping, navigation):
        checkpoint["roles"]["offline_agent"]["untrusted_extra_field"] = "same"

    evidence = build_runtime_continuity_evidence(
        mapping,
        navigation,
        required_roles=("gazebo_server", "offline_agent"),
    )

    assert evidence["passed"] is False
    assert (
        evidence["checks"]["offline_agent_identity_schema_valid"] is False
    )


def test_continuity_requires_the_exact_audited_role_set() -> None:
    mapping = RuntimeCheckpoint.capture(
        "mapping_ready",
        {
            "gazebo_server": os.getpid(),
            "offline_agent": os.getpid(),
            "untracked_helper": os.getpid(),
        },
        wall_ns=100,
    )
    navigation = RuntimeCheckpoint.capture(
        "navigation_ready",
        {
            "gazebo_server": os.getpid(),
            "offline_agent": os.getpid(),
            "untracked_helper": os.getpid(),
        },
        wall_ns=200,
    )

    evidence = build_runtime_continuity_evidence(
        mapping,
        navigation,
        required_roles=("gazebo_server", "offline_agent"),
    )

    assert evidence["passed"] is False
    assert evidence["checks"]["required_roles_present"] is True
    assert evidence["checks"]["role_sets_exact"] is False


def test_runtime_probe_requires_agent_role_selected_by_mode() -> None:
    roles = {
        "gazebo_server": os.getpid(),
        "offline_agent": os.getpid(),
        "robot_state_publisher": os.getpid(),
    }
    probe = RuntimeContinuityProbe.from_flags(
        enabled=True,
        require_rviz=False,
        agent_mode="offline",
    )
    probe.mapping_ready = RuntimeCheckpoint.capture(
        "mapping_ready",
        roles,
        wall_ns=100,
    )
    probe.navigation_ready = RuntimeCheckpoint.capture(
        "navigation_ready",
        roles,
        wall_ns=200,
    )
    report: dict[str, object] = {"checks": {}, "passed": True}

    probe.attach_to(report)

    assert report["runtime_continuity"]["required_roles"] == [
        "gazebo_server",
        "offline_agent",
        "robot_state_publisher",
    ]
    assert report["checks"]["runtime_continuity"] is True


def _strict_report_with_continuity(evidence: dict[str, object]) -> dict:
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
        "runtime_continuity": bool(evidence["passed"]),
    }
    return {
        "schema_version": 4,
        "evidence_kind": "unknown_world_slam_nav_dynamic_replan",
        "passed": bool(evidence["passed"]),
        "session_id": "persistent-session",
        "checks": checks,
        "map_quality": {
            "passed": True,
            "metrics": {
                "reachable_free_coverage_ratio": 0.95,
                "region_coverage_ratios": {"main": 0.93},
            },
        },
        "frontier": {"passed": True},
        "return_to_start": {"passed": True},
        "localization": {
            "passed": True,
            "metrics": {"position_error_p95_m": 0.11},
        },
        "sampled_navigation": {
            "passed": True,
            "metrics": {"goal_count": 3},
        },
        "dynamic_navigation": {"passed": True},
        "runtime_continuity": evidence,
    }


def test_persistent_verifier_rejects_a_restarted_runtime_role() -> None:
    mapping = RuntimeCheckpoint.capture(
        "mapping_ready",
        {
            "gazebo_server": os.getpid(),
            "offline_agent": os.getpid(),
            "robot_state_publisher": os.getpid(),
        },
        wall_ns=100,
    )
    navigation = RuntimeCheckpoint.capture(
        "navigation_ready",
        {
            "gazebo_server": os.getpid(),
            "offline_agent": os.getpid(),
            "robot_state_publisher": os.getpid(),
        },
        wall_ns=200,
    )
    changed = navigation.as_evidence()
    changed["roles"]["gazebo_server"]["start_ticks"] += 1
    evidence = build_runtime_continuity_evidence(
        mapping.as_evidence(),
        changed,
        required_roles=(
            "gazebo_server",
            "offline_agent",
            "robot_state_publisher",
        ),
    )
    report = _strict_report_with_continuity(evidence)

    with pytest.raises(ValueError, match="runtime continuity"):
        verify_persistent_runtime_report(
            report,
            expected_session_id="persistent-session",
            require_rviz=False,
            agent_mode="offline",
        )


def test_persistent_verifier_recomputes_continuity_instead_of_trusting_flags() -> None:
    mapping = RuntimeCheckpoint.capture(
        "mapping_ready",
        {
            "gazebo_server": os.getpid(),
            "offline_agent": os.getpid(),
            "robot_state_publisher": os.getpid(),
        },
        wall_ns=100,
    )
    navigation = RuntimeCheckpoint.capture(
        "navigation_ready",
        {
            "gazebo_server": os.getpid(),
            "offline_agent": os.getpid(),
            "robot_state_publisher": os.getpid(),
        },
        wall_ns=200,
    )
    evidence = build_runtime_continuity_evidence(
        mapping,
        navigation,
        required_roles=(
            "gazebo_server",
            "offline_agent",
            "robot_state_publisher",
        ),
    )
    # 模拟报告被错误后处理：布尔标志仍是 PASS，但导航阶段身份已被替换。
    evidence["checkpoints"]["navigation_ready"]["roles"]["gazebo_server"][
        "start_ticks"
    ] += 1
    report = _strict_report_with_continuity(evidence)

    with pytest.raises(ValueError, match="runtime continuity"):
        verify_persistent_runtime_report(
            report,
            expected_session_id="persistent-session",
            require_rviz=False,
            agent_mode="offline",
        )


def test_persistent_verifier_rejects_the_wrong_agent_role() -> None:
    roles = {
        "gazebo_server": os.getpid(),
        "offline_agent": os.getpid(),
        "robot_state_publisher": os.getpid(),
    }
    evidence = build_runtime_continuity_evidence(
        RuntimeCheckpoint.capture("mapping_ready", roles, wall_ns=100),
        RuntimeCheckpoint.capture("navigation_ready", roles, wall_ns=200),
        required_roles=roles,
    )
    report = _strict_report_with_continuity(evidence)

    with pytest.raises(ValueError, match="runtime continuity"):
        verify_persistent_runtime_report(
            report,
            expected_session_id="persistent-session",
            require_rviz=False,
            agent_mode="online",
        )


def test_persistent_verifier_rejects_unknown_continuity_schema() -> None:
    roles = {
        "gazebo_server": os.getpid(),
        "offline_agent": os.getpid(),
        "robot_state_publisher": os.getpid(),
    }
    evidence = build_runtime_continuity_evidence(
        RuntimeCheckpoint.capture("mapping_ready", roles, wall_ns=100),
        RuntimeCheckpoint.capture("navigation_ready", roles, wall_ns=200),
        required_roles=roles,
    )
    evidence["schema_version"] = 99
    report = _strict_report_with_continuity(evidence)

    with pytest.raises(ValueError, match="runtime continuity schema"):
        verify_persistent_runtime_report(
            report,
            expected_session_id="persistent-session",
            require_rviz=False,
            agent_mode="offline",
        )


def test_persistent_verifier_recomputes_checkpoint_time_order() -> None:
    roles = {
        "gazebo_server": os.getpid(),
        "offline_agent": os.getpid(),
        "robot_state_publisher": os.getpid(),
    }
    evidence = build_runtime_continuity_evidence(
        RuntimeCheckpoint.capture("mapping_ready", roles, wall_ns=100),
        RuntimeCheckpoint.capture("navigation_ready", roles, wall_ns=200),
        required_roles=roles,
    )
    # 模拟后处理只篡改原始时间戳而保留旧 PASS/checks。
    evidence["checkpoints"]["navigation_ready"]["wall_ns"] = 50
    report = _strict_report_with_continuity(evidence)

    with pytest.raises(ValueError, match="runtime continuity"):
        verify_persistent_runtime_report(
            report,
            expected_session_id="persistent-session",
            require_rviz=False,
            agent_mode="offline",
        )


def test_persistent_commands_add_only_explicit_runtime_contract(tmp_path: Path) -> None:
    profile = UnknownWorldRunProfile.synthetic()
    strict_orchestrator = _build_orchestrator_command(
        profile=profile,
        workspace=tmp_path,
        map_prefix=tmp_path / "strict-map",
        mission_plan=tmp_path / "mission.yaml",
    )
    persistent_orchestrator = _build_orchestrator_command(
        profile=profile,
        workspace=tmp_path,
        map_prefix=tmp_path / "persistent-map",
        mission_plan=tmp_path / "mission.yaml",
        persistent_runtime_enabled=True,
    )
    persistent_probe = _build_probe_command(
        profile=profile,
        workspace=tmp_path,
        report_path=tmp_path / "report.json",
        transition_timeout_s=10.0,
        gate_timeout_s=20.0,
        heartbeat_s=3.0,
        runtime_log=tmp_path / "runtime.log",
        session_id="persistent-session",
        session_started_ns=123,
        world_file=tmp_path / "world.sdf",
        mission_plan=tmp_path / "mission.yaml",
        dynamic_scenario=tmp_path / "dynamic.json",
        scene_spec=tmp_path / "scene.yaml",
        truth_map=tmp_path / "truth.yaml",
        voice_trigger_timeout_s=0.0,
        persistent_runtime_enabled=True,
        require_rviz_continuity=True,
    )

    assert "authority_gate_enabled:=true" not in strict_orchestrator
    assert "persistent_runtime_enabled:=true" not in strict_orchestrator
    assert "authority_gate_enabled:=true" in persistent_orchestrator
    assert "persistent_runtime_enabled:=true" in persistent_orchestrator
    assert "--require-runtime-continuity" in persistent_probe
    assert "--require-rviz-continuity" in persistent_probe
    agent_mode_index = persistent_probe.index("--agent-mode")
    assert persistent_probe[agent_mode_index + 1] == "offline"


class _AuthoritySession:
    def __init__(self, tmp_path: Path) -> None:
        self.tmp_path = tmp_path
        self.spawned: list[tuple[str, tuple[str, ...], Path]] = []
        self.runs: list[tuple[tuple[str, ...], float]] = []

    def log_path(self, role: str) -> Path:
        return self.tmp_path / f"{role}.log"

    def spawn(self, role, argv, *, log_path):
        self.spawned.append((role, tuple(argv), log_path))

    def run(self, argv, *, timeout_s):
        self.runs.append((tuple(argv), timeout_s))
        return 0


def test_persistent_authority_starts_exactly_one_manager_then_bootstraps(
    tmp_path: Path,
) -> None:
    session = _AuthoritySession(tmp_path)

    _start_persistent_authority(session)

    assert len(session.spawned) == 1
    assert session.spawned[0][0] == "control_authority"
    manager_argv = session.spawned[0][1]
    assert manager_argv[:4] == (
        "ros2",
        "run",
        "embodied_agent_cpp",
        "control_authority",
    )
    assert "bootstrap_quiescence_acknowledged:=true" in manager_argv
    assert session.runs == [
        (
            (
                "ros2",
                "run",
                "embodied_slam_tools",
                "control_authority_bootstrap",
            ),
            12.0,
        )
    ]


def test_cleanup_covers_the_persistent_showcase_process_tree() -> None:
    cleanup = (ROOT / "scripts" / "cleanup_simulation_processes.sh").read_text(
        encoding="utf-8"
    )

    assert "persistent_voice_nav_base.launch.py" in cleanup
    assert "persistent_mapping_stage.launch.py" in cleanup
    assert "persistent_navigation_stage.launch.py" in cleanup
    assert "tools.acceptance.scenarios.showcase_gazebo_e2e" in cleanup
    assert "/robot_state_publisher/robot_state_publisher" in cleanup
    assert "/rviz2/rviz2" in cleanup


def test_showcase_mode_is_internal_and_dispatches_one_handler() -> None:
    mode = MODE_BY_NAME["showcase-gazebo-e2e"]
    assert mode.public is False
    assert mode.handler == "accept_showcase_gazebo_e2e"

    class Runner:
        calls: list[tuple[str, list[str]]] = []

        def run(self, selected, arguments):
            self.calls.append((selected.name, list(arguments)))
            return 0

    runner = Runner()
    assert acceptance_main(
        ["showcase-gazebo-e2e"],
        runner=runner,
        stdout=io.StringIO(),
        stderr=io.StringIO(),
    ) == 0
    assert runner.calls == [("showcase-gazebo-e2e", [])]


def test_showcase_wrapper_reuses_strict_runner_with_persistent_flag(
    monkeypatch,
) -> None:
    captured: list[tuple[UnknownWorldRunProfile, bool]] = []

    def fake_run(
        profile: UnknownWorldRunProfile,
        *,
        persistent_runtime_enabled: bool = False,
    ) -> int:
        captured.append((profile, persistent_runtime_enabled))
        return 17

    monkeypatch.setattr(showcase_gazebo_e2e, "run", fake_run)

    assert showcase_gazebo_e2e.main() == 17
    assert captured[0][0] == UnknownWorldRunProfile.synthetic()
    assert captured[0][1] is True


def test_probe_cli_exposes_persistent_continuity_as_opt_in() -> None:
    strict = build_parser().parse_args(["--output", "/tmp/strict.json"])
    persistent = build_parser().parse_args(
        [
            "--output",
            "/tmp/persistent.json",
            "--require-runtime-continuity",
            "--require-rviz-continuity",
        ]
    )

    assert strict.require_runtime_continuity is False
    assert strict.require_rviz_continuity is False
    assert persistent.require_runtime_continuity is True
    assert persistent.require_rviz_continuity is True
