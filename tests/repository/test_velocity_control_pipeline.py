"""多输入速度仲裁到 Nav2 Collision Monitor 的静态契约测试。"""

import ast
import xml.etree.ElementTree as ET

import yaml

from repository_test_support import ROOT


SIMULATION_ROOT = ROOT / "src" / "embodied_simulation"


def test_twist_mux_only_arbitrates_nav2_and_voice_autonomy():
    config = yaml.safe_load(
        (SIMULATION_ROOT / "config" / "twist_mux.yaml").read_text(
            encoding="utf-8"
        )
    )["twist_mux"]["ros__parameters"]

    assert config["use_stamped"] is False
    topics = config["topics"]
    assert topics["nav2"] == {
        "topic": "/control/nav2/cmd_vel",
        "timeout": 0.5,
        "priority": 50,
    }
    assert topics["voice"] == {
        "topic": "/control/voice/cmd_vel",
        "timeout": 0.5,
        "priority": 70,
    }
    assert set(topics) == {"nav2", "voice"}
    assert "locks" not in config


def test_manual_executor_velocity_is_remapped_at_launch_boundary():
    launch = (
        SIMULATION_ROOT / "launch" / "simulation_control.launch.py"
    ).read_text(encoding="utf-8")

    assert '"cmd_vel_topic"' in launch
    # standalone LifecycleNode 与 composable component 必须遵循同一出口契约。
    assert launch.count('remappings=[("cmd_vel", cmd_vel_topic)]') == 2


def test_nav2_velocity_pipeline_keeps_collision_monitor_as_final_output():
    launch = (
        SIMULATION_ROOT / "launch" / "voice_nav2_turtlebot3.launch.py"
    ).read_text(encoding="utf-8")

    assert "SetRemap(" in launch
    assert 'src="cmd_vel_smoothed"' in launch
    assert 'dst="/control/nav2/cmd_vel"' in launch
    assert 'package="twist_mux"' in launch
    # 兼容模式直达 selected；控制权模式先聚合 autonomy，再由 C++ gate 选择。
    assert '("/cmd_vel_out", "/control/selected/cmd_vel")' in launch
    assert '("/cmd_vel_out", "/control/autonomy/cmd_vel")' in launch
    assert "validate_control_authority_owner" in launch
    assert "quiescence coordinator" in launch
    assert "control_authority_manager_node(" not in launch
    assert 'executable="velocity_authority_gate"' in launch
    assert 'DeclareLaunchArgument("control_authority_enabled"' in launch
    assert '"control_authority_manager_enabled"' in launch
    assert 'default_value="false"' in launch
    assert "condition=UnlessCondition(control_authority_enabled)" in launch
    assert launch.count("condition=IfCondition(control_authority_enabled)") >= 2
    assert '"cmd_vel_in_topic": "/control/selected/cmd_vel"' in launch
    assert '"cmd_vel_out_topic": "/cmd_vel"' in launch
    assert '"cmd_vel_topic": "/control/voice/cmd_vel"' in launch


def test_nav2_launch_has_no_duplicate_literal_parameter_keys():
    """Python 会静默覆盖重复 dict key；ASR/VAD 参数不能靠运行时才发现丢失。"""

    path = SIMULATION_ROOT / "launch" / "voice_nav2_turtlebot3.launch.py"
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))

    for node in ast.walk(tree):
        if not isinstance(node, ast.Dict):
            continue
        literal_keys = [
            key.value
            for key in node.keys
            if isinstance(key, ast.Constant) and isinstance(key.value, str)
        ]
        assert len(literal_keys) == len(set(literal_keys)), (
            f"duplicate literal dict key near line {node.lineno}"
        )


def test_showcase_session_owns_one_manager_and_children_only_own_the_gate():
    showcase = (ROOT / "scripts" / "voice_slam_nav_showcase.sh").read_text(
        encoding="utf-8"
    )
    child = (ROOT / "scripts" / "continuous_nav2_voice_control.sh").read_text(
        encoding="utf-8"
    )

    assert (
        'CONTROL_AUTHORITY_MANAGER_ENABLED="${CONTROL_AUTHORITY_MANAGER_ENABLED:-false}"'
        in child
    )
    assert (
        'add_launch_arg control_authority_manager_enabled "$CONTROL_AUTHORITY_MANAGER_ENABLED"'
        in child
    )
    assert "ros2 topic type /slam/session_state" in child
    assert (
        'SESSION_CONTROL_AUTHORITY_MANAGER_ENABLED="$CONTROL_AUTHORITY_MANAGER_ENABLED"'
        in showcase
    )
    assert "start_session_authority_manager" in showcase
    assert "export CONTROL_AUTHORITY_MANAGER_ENABLED=false" in showcase
    assert "setsid ros2 run embodied_agent_cpp control_authority" in showcase
    assert 'bash "$WORKSPACE/scripts/cleanup_simulation_processes.sh"' in showcase
    assert "-p bootstrap_quiescence_acknowledged:=true" in showcase
    assert "ros2 run embodied_slam_tools control_authority_bootstrap" in showcase
    assert "auto showcase must own its session-level" in showcase
    assert "cleanup_session_authority_manager" in showcase
    # 键盘需要独占 TTY，只能由 scripts/keyboard_control.sh 在第二终端启动。
    assert "keyboard_teleop" not in showcase
    assert "keyboard_control.sh" not in showcase
    assert "keyboard_teleop" not in child
    assert "keyboard_control.sh" not in child


def test_velocity_mux_is_an_installed_runtime_dependency():
    package = ET.parse(SIMULATION_ROOT / "package.xml").getroot()
    exec_dependencies = {
        element.text for element in package.findall("exec_depend")
    }
    cmake = (SIMULATION_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "twist_mux" in exec_dependencies
    assert "install(DIRECTORY config launch maps rviz worlds" in cmake


def test_authority_manager_heartbeats_typed_state_without_boolean_locks():
    manager = (
        ROOT
        / "src"
        / "embodied_agent_cpp"
        / "src"
        / "control_authority_node.cpp"
    ).read_text(encoding="utf-8")

    assert '"state_heartbeat_ms"' in manager
    assert "state_publisher_->publish(to_message(snapshot))" in manager
    assert "message.manager_epoch = snapshot.manager_epoch" in manager
    assert "manual_lock_topic" not in manager
    assert "estop_lock_topic" not in manager


def test_keyboard_operator_contract_is_fail_safe_and_explained():
    script = (ROOT / "scripts" / "keyboard_control.sh").read_text(
        encoding="utf-8"
    )
    node = (
        ROOT
        / "src"
        / "embodied_agent_cpp"
        / "src"
        / "keyboard_teleop_node.cpp"
    ).read_text(encoding="utf-8")

    assert 'KEYBOARD_DEADMAN_TIMEOUT_MS:-600' in script
    assert "flock --nonblock 9" in script
    assert "embodied-agent-keyboard-${UID}-${ROS_DOMAIN_ID:-0}.lock" in script
    assert "Space：进入 HOLD" in script
    assert "再次按 R 恢复自动控制" in script
    assert "SetAuthority::Request::ENTER_HOLD, \"keyboard_soft_stop\"" in node
    assert "authority.active_source == requester_" in node
    assert "transition_sequence" in node
