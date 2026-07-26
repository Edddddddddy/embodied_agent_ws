"""公开 Gazebo 验收必须复用受控 ROS overlay 会话。"""

from __future__ import annotations

from pathlib import Path

from tools.acceptance.scenarios import gazebo_typed_action
from tools.acceptance.session import RosEnvironmentIsolation


ROOT = Path(__file__).resolve().parents[2]


def test_public_gazebo_handler_uses_isolated_scenario():
    """不能直接继承宿主 Nav2 overlay，否则 ABI 漂移会让 Lifecycle 全部退出。"""

    common = (
        ROOT / "tools/acceptance/handlers/common.sh"
    ).read_text(encoding="utf-8")

    assert (
        "python3 -m tools.acceptance.scenarios.gazebo_typed_action"
        in common
    )


def test_gazebo_smoke_surfaces_launch_log_when_readiness_fails():
    """Lifecycle 启动失败时不能只给两个图谱 blocker，必须同时展示根因日志。"""

    smoke = (
        ROOT / "scripts/smoke_test_gazebo_typed_action.sh"
    ).read_text(encoding="utf-8")

    assert (
        'if ! python "$WORKSPACE/scripts/simulation_readiness_check.py"'
        in smoke
    )
    assert "readiness failed; launch log follows" in smoke


def test_gazebo_probe_requires_fresh_stable_zero_velocity_after_stop():
    """stop ACK 只代表指令被接收，正式验收还必须证明底盘控制量已经归零。"""

    probe = (
        ROOT / "tools/acceptance/probes/control/gazebo_motion.py"
    ).read_text(encoding="utf-8")

    assert 'self.create_subscription(Twist, "/cmd_vel"' in probe
    assert "final_velocity_is_zero(after=stop_requested_at)" in probe
    assert '"fresh_after_stop"' in probe
    assert '"stable_zero": True' in probe


def test_gazebo_scenario_runs_smoke_inside_isolated_session(
    monkeypatch,
    tmp_path,
):
    observed: dict[str, object] = {}

    class FakeSession:
        def __init__(self, config):
            observed["config"] = config

        def __enter__(self):
            return self

        def run(self, argv, *, timeout_s):
            observed["argv"] = tuple(argv)
            observed["timeout_s"] = timeout_s
            return 0

        def __exit__(self, _error_type, _error, _traceback):
            return False

    monkeypatch.setattr(gazebo_typed_action, "AcceptanceSession", FakeSession)

    assert gazebo_typed_action.run(workspace=tmp_path) == 0

    config = observed["config"]
    assert isinstance(config.ros_environment_isolation, RosEnvironmentIsolation)
    assert "embodied_simulation" in (
        config.ros_environment_isolation.required_packages
    )
    assert "embodied_agent_interfaces" in (
        config.ros_environment_isolation.required_packages
    )
    assert config.name == "gazebo-typed-action"
    assert config.timeout_s == 110.0
    assert config.inherit_ros_domain_id is False
    assert config.artifact_root == (
        tmp_path.resolve() / "logs/acceptance/gazebo_typed_action"
    )
    assert config.environment["EMBODIED_ACTIVE_WORKSPACE"] == str(
        tmp_path.resolve()
    )
    assert observed["argv"] == (
        "bash",
        "scripts/smoke_test_gazebo_typed_action.sh",
    )
    assert observed["timeout_s"] == 95.0
