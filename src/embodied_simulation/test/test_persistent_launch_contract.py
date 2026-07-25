"""持久 Gazebo 会话的 launch seam 契约测试.

测试只加载 LaunchDescription，不启动 Gazebo/ROS 进程。这样可以在 CI 中验证
base/mapping/navigation 三个公开入口的职责，不把测试绑死到内部节点排列顺序。
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import UnlessCondition
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import LifecycleNode, Node


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_ROOT = PACKAGE_ROOT / "launch"


def _load_module(filename: str):
    path = LAUNCH_ROOT / filename
    spec = importlib.util.spec_from_file_location(path.stem, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_description(filename: str) -> LaunchDescription:
    module = _load_module(filename)
    description = module.generate_launch_description()
    assert isinstance(description, LaunchDescription)
    return description


def _declared_arguments(description: LaunchDescription) -> set[str]:
    return {
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }


def _argument_defaults(description: LaunchDescription) -> dict[str, str]:
    context = LaunchContext()
    defaults = {}
    for entity in description.entities:
        if not isinstance(entity, DeclareLaunchArgument):
            continue
        defaults[entity.name] = perform_substitutions(
            context,
            normalize_to_list_of_substitutions(entity.default_value or []),
        )
    return defaults


def _argument_default(description: LaunchDescription, name: str) -> str:
    """读取一个字面量默认值，避免其他动态参数干扰契约断言."""

    entity = next(
        item
        for item in description.entities
        if isinstance(item, DeclareLaunchArgument) and item.name == name
    )
    return perform_substitutions(
        LaunchContext(),
        normalize_to_list_of_substitutions(entity.default_value or []),
    )


def _contract_entities(description: LaunchDescription):
    """遍历本项目的 GroupAction，但不展开第三方 include 的内部实现."""
    pending = list(description.entities)
    while pending:
        entity = pending.pop(0)
        yield entity
        if isinstance(entity, GroupAction):
            pending[0:0] = list(entity.get_sub_entities())


def _include_filename(action: IncludeLaunchDescription) -> str:
    source = action.launch_description_source
    substitutions = source._LaunchDescriptionSource__location
    return Path(perform_substitutions(LaunchContext(), substitutions)).name


def _include_arguments(
    action: IncludeLaunchDescription,
    launch_configurations: dict[str, str] | None = None,
) -> dict[str, str]:
    context = LaunchContext()
    context.launch_configurations.update(launch_configurations or {})
    arguments = {}
    for key, value in action.launch_arguments:
        name = perform_substitutions(
            context, normalize_to_list_of_substitutions(key)
        )
        try:
            arguments[name] = perform_substitutions(
                context, normalize_to_list_of_substitutions(value)
            )
        except Exception:  # 未赋值的 LaunchConfiguration 是合法动态参数
            arguments[name] = "<dynamic>"
    return arguments


def _literal_node_executable(action: Node) -> str:
    return perform_substitutions(
        LaunchContext(),
        normalize_to_list_of_substitutions(action.node_executable),
    )


def test_persistent_base_launch_is_loadable_and_exposes_session_interface():
    description = _load_description("persistent_voice_nav_base.launch.py")

    arguments = _declared_arguments(description)
    assert {
        "agent_type",
        "provider_mode",
        "use_rviz",
        "headless",
        "world",
        "params_file",
        "control_authority_enabled",
    } <= arguments
    assert "executor_plugin" not in arguments
    assert "map" not in arguments
    assert "readiness_stale_timeout_s" not in arguments


def test_persistent_base_owns_nav2_common_but_no_stage_or_authority_manager():
    description = _load_description("persistent_voice_nav_base.launch.py")
    entities = list(_contract_entities(description))
    includes = [
        entity for entity in entities if isinstance(entity, IncludeLaunchDescription)
    ]
    include_by_name = {_include_filename(action): action for action in includes}

    assert "bringup_launch.py" in include_by_name
    nav2_arguments = _include_arguments(include_by_name["bringup_launch.py"])
    assert nav2_arguments["slam"].lower() == "false"
    assert nav2_arguments["use_localization"].lower() == "false"
    assert "slam_launch.py" not in include_by_name
    assert "localization_launch.py" not in include_by_name
    assert "simulation_control.launch.py" not in include_by_name

    executables = {
        _literal_node_executable(entity)
        for entity in entities
        if isinstance(entity, Node)
    }
    assert "control_authority" not in executables
    assert "simulation_control_node" not in executables


def test_persistent_base_normalizes_booleans_for_nav2_python_expressions():
    description = _load_description("persistent_voice_nav_base.launch.py")
    bringup = next(
        entity
        for entity in _contract_entities(description)
        if isinstance(entity, IncludeLaunchDescription)
        and _include_filename(entity) == "bringup_launch.py"
    )
    arguments = _include_arguments(
        bringup,
        {
            "use_composition": "true",
            "use_respawn": "false",
        },
    )

    # Nav2 官方 launch 通过 PythonExpression 组合这些参数，必须收到
    # Python 可识别的 True/False，而不是 ROS 常见的小写 true/false。
    assert arguments["slam"] == "False"
    assert arguments["use_localization"] == "False"
    assert arguments["use_composition"] == "True"


def test_persistent_base_owns_simulator_robot_rviz_and_agent_lifetime():
    description = _load_description("persistent_voice_nav_base.launch.py")
    entities = list(_contract_entities(description))
    include_names = {
        _include_filename(entity)
        for entity in entities
        if isinstance(entity, IncludeLaunchDescription)
    }
    executables = {
        _literal_node_executable(entity)
        for entity in entities
        if isinstance(entity, Node)
    }

    assert {
        "spawn_tb3.launch.py",
        "rviz_launch.py",
        "online_agent.launch.py",
        "offline_agent.launch.py",
    } <= include_names
    assert "robot_state_publisher" in executables
    # launch_agent=false 用于确定性 stage 验收；仍需保留 typed 安全边界。
    assert "action_guard" in executables


def test_persistent_base_headless_condition_does_not_evaluate_python_booleans():
    description = _load_description("persistent_voice_nav_base.launch.py")
    gazebo_includes = [
        entity
        for entity in _contract_entities(description)
        if isinstance(entity, IncludeLaunchDescription)
        and _include_filename(entity) == "gz_sim.launch.py"
    ]

    assert len(gazebo_includes) == 1
    # LaunchConfiguration 会展开为小写 true/false；若拼成 PythonExpression
    # 的 ``not true``，Python 会把 true 当未定义变量并让整套 base 退出。
    assert isinstance(gazebo_includes[0].condition, UnlessCondition)


def test_persistent_base_preserves_live_voice_frontend_interface():
    description = _load_description("persistent_voice_nav_base.launch.py")
    arguments = _declared_arguments(description)
    entities = list(_contract_entities(description))
    include_by_name = {
        _include_filename(entity): entity
        for entity in entities
        if isinstance(entity, IncludeLaunchDescription)
    }
    expected = {
        "capture_enabled",
        "speaker_enabled",
        "vad_provider",
        "speech_start_threshold",
        "vad_speech_start_ms",
        "speech_end_silence_s",
        "min_utterance_ms",
        "max_utterance_s",
        "silero_model_path",
        "silero_use_onnx",
        "silero_threshold",
        "silero_end_threshold",
        "kws_provider",
        "audio_enhancer",
        "aec_enabled",
        "noise_suppression_enabled",
        "auto_gain_enabled",
        "asr_hotwords_score",
    }

    assert expected <= arguments
    online_arguments = _include_arguments(
        include_by_name["online_agent.launch.py"]
    )
    offline_arguments = _include_arguments(
        include_by_name["offline_agent.launch.py"]
    )
    assert expected - {"asr_hotwords_score"} <= set(online_arguments)
    assert expected <= set(offline_arguments)


def test_persistent_base_owns_velocity_safety_and_typed_action_bridge():
    description = _load_description("persistent_voice_nav_base.launch.py")
    entities = list(_contract_entities(description))
    executables = {
        _literal_node_executable(entity)
        for entity in entities
        if isinstance(entity, Node)
    }
    source = (
        LAUNCH_ROOT / "persistent_voice_nav_base.launch.py"
    ).read_text(encoding="utf-8")

    assert {
        "twist_mux",
        "velocity_authority_gate",
        "typed_action_bridge",
    } <= executables
    # readiness 聚合器必须跟随 stage 重建；若常驻在 base，建图 executor
    # 的旧心跳可能在切换后冒充导航 executor，造成假就绪。
    assert "system_readiness_node" not in executables
    assert 'src="cmd_vel_smoothed"' in source
    assert 'dst="/control/nav2/cmd_vel"' in source
    assert '("/cmd_vel_out", "/control/autonomy/cmd_vel")' in source
    assert '"cmd_vel_in_topic": "/control/selected/cmd_vel"' in source
    assert '"cmd_vel_out_topic": "/cmd_vel"' in source
    # 官方 Nav2 bringup 还会启动 docking_server；它的硬编码 cmd_vel 必须按
    # 节点作用域移出最终底盘 topic，保证 Collision Monitor 是唯一写者。
    assert 'src="docking_server:/cmd_vel"' in source
    assert 'dst="/control/docking/cmd_vel"' in source
    # Collision Monitor 默认只在停车后的短窗口重复发布零速。持久会话必须
    # 覆盖整场演示，否则稍后再发 typed STOP 时无法形成“本次停车之后”的
    # 新鲜最终输出证据。
    assert float(
        _argument_default(
            description, "persistent_zero_heartbeat_timeout_s"
        )
    ) >= 3600.0
    assert (
        '"stop_pub_timeout": persistent_zero_heartbeat_timeout_s'
        in source
    )
    assert "persistent_zero_heartbeat_timeout_s" in source


def test_monolithic_voice_nav_keeps_typed_stop_evidence_fresh():
    """长任务静止后再次 STOP，最终速度边界仍必须发布本次零速证据。"""

    description = _load_description("voice_nav2_turtlebot3.launch.py")
    source = (
        LAUNCH_ROOT / "voice_nav2_turtlebot3.launch.py"
    ).read_text(encoding="utf-8")

    # Collision Monitor 官方默认仅转发停车后 2 秒内的零速。未知环境建图会
    # 远长于该窗口；若不覆盖，typed STOP 虽成功，/cmd_vel 却不会产生新帧。
    assert float(
        _argument_default(
            description, "persistent_zero_heartbeat_timeout_s"
        )
    ) >= 3600.0
    assert (
        '"stop_pub_timeout": persistent_zero_heartbeat_timeout_s'
        in source
    )
    assert 'src="docking_server:/cmd_vel"' in source
    assert 'dst="/control/docking/cmd_vel"' in source


def test_typed_bridge_lifecycle_is_owned_by_session_orchestrator():
    description = _load_description("persistent_voice_nav_base.launch.py")
    entities = list(_contract_entities(description))
    source = (
        LAUNCH_ROOT / "persistent_voice_nav_base.launch.py"
    ).read_text(encoding="utf-8")
    bridge = next(
        entity
        for entity in entities
        if isinstance(entity, LifecycleNode)
        and _literal_node_executable(entity) == "typed_action_bridge"
    )

    # launch_ros autostart 依赖一次性 transition_event 串联
    # configure→activate；高负载冷启动时订阅发现晚于 service，事件可能丢失。
    # 持久会话改由唯一 SessionOrchestrator 通过状态/服务闭环推进生命周期。
    assert bridge.node_autostart is False
    assert "typed_action_bridge_lifecycle_manager" not in source
    assert "TimerAction(" not in source


def test_mapping_stage_only_owns_slam_provider_and_gazebo_executor():
    description = _load_description("persistent_mapping_stage.launch.py")
    arguments = _declared_arguments(description)
    entities = list(_contract_entities(description))
    includes = [
        entity for entity in entities if isinstance(entity, IncludeLaunchDescription)
    ]
    include_by_name = {_include_filename(action): action for action in includes}
    executables = {
        _literal_node_executable(entity)
        for entity in entities
        if isinstance(entity, Node)
    }

    assert set(include_by_name) == {
        "slam_launch.py",
        "simulation_control.launch.py",
    }
    assert "system_readiness_node" in executables
    assert "dynamic_obstacle_tracker_node" not in executables
    assert "readiness_required_components" in arguments
    source = (
        LAUNCH_ROOT / "persistent_mapping_stage.launch.py"
    ).read_text(encoding="utf-8")
    assert '"profile": "persistent_mapping_stage"' in source
    executor_arguments = _include_arguments(
        include_by_name["simulation_control.launch.py"]
    )
    assert executor_arguments["executor_plugin"] == (
        "embodied_simulation/GazeboRobotExecutor"
    )
    assert executor_arguments["use_typed_actions"].lower() == "false"
    assert executor_arguments["autostart"].lower() == "false"
    assert executor_arguments["lifecycle_manager_enabled"].lower() == "false"
    assert executor_arguments["cmd_vel_topic"] == "/control/voice/cmd_vel"
    assert "world" not in arguments
    assert "agent_type" not in arguments
    assert "map" not in arguments


def test_navigation_stage_only_owns_localization_and_nav2_executor():
    description = _load_description("persistent_navigation_stage.launch.py")
    arguments = _declared_arguments(description)
    defaults = _argument_defaults(description)
    entities = list(_contract_entities(description))
    includes = [
        entity for entity in entities if isinstance(entity, IncludeLaunchDescription)
    ]
    include_by_name = {_include_filename(action): action for action in includes}
    executables = {
        _literal_node_executable(entity)
        for entity in entities
        if isinstance(entity, Node)
    }

    assert set(include_by_name) == {
        "localization_launch.py",
        "simulation_control.launch.py",
    }
    assert "system_readiness_node" in executables
    # 动态障碍只属于定位/导航阶段，不能在建图时把演示障碍写进地图。
    assert "dynamic_obstacle_tracker_node" in executables
    assert "enable_dynamic_obstacle_layer" in arguments
    assert "readiness_required_components" in arguments
    source = (
        LAUNCH_ROOT / "persistent_navigation_stage.launch.py"
    ).read_text(encoding="utf-8")
    assert '"profile": "persistent_navigation_stage"' in source
    localization_arguments = _include_arguments(
        include_by_name["localization_launch.py"]
    )
    assert localization_arguments["container_name"] == "nav2_container"
    executor_arguments = _include_arguments(
        include_by_name["simulation_control.launch.py"]
    )
    assert executor_arguments["executor_plugin"] == (
        "embodied_simulation/Nav2RobotExecutor"
    )
    assert executor_arguments["use_typed_actions"].lower() == "false"
    assert executor_arguments["autostart"].lower() == "false"
    assert executor_arguments["lifecycle_manager_enabled"].lower() == "false"
    # 定位 provider 必须由 stage 子进程持有；装进 base 的常驻 container 后
    # StageProcessManager 仅终止 launch 进程无法保证卸载组件。
    assert defaults["use_composition"].lower() == "false"
    assert "map" in arguments
    assert "world" not in arguments
    assert "agent_type" not in arguments


def test_simulation_control_allows_an_external_lifecycle_owner():
    description = _load_description("simulation_control.launch.py")
    arguments = _declared_arguments(description)
    source = (
        LAUNCH_ROOT / "simulation_control.launch.py"
    ).read_text(encoding="utf-8")

    assert "lifecycle_manager_enabled" in arguments
    assert "condition=IfCondition(lifecycle_manager_enabled)" in source


def test_navigation_stage_normalizes_booleans_for_nav2_python_expressions():
    description = _load_description("persistent_navigation_stage.launch.py")
    localization = next(
        entity
        for entity in _contract_entities(description)
        if isinstance(entity, IncludeLaunchDescription)
        and _include_filename(entity) == "localization_launch.py"
    )
    arguments = _include_arguments(
        localization,
        {
            "use_composition": "false",
            "use_respawn": "true",
        },
    )

    # Nav2 的 localization_launch.py 会拼接 PythonExpression。若把 ROS
    # 常见的小写 false 直接传入，就会求值成未定义变量并让整个导航阶段退出。
    assert arguments["use_composition"] == "False"
    assert arguments["use_respawn"] == "True"


def test_persistent_base_starts_gazebo_only_after_world_xacro_finishes():
    source = (
        LAUNCH_ROOT / "persistent_voice_nav_base.launch.py"
    ).read_text(encoding="utf-8")

    assert "OnProcessExit" in source
    assert "target_action=world_xacro" in source
    assert "_handle_world_xacro_exit" in source


def test_world_xacro_failure_never_starts_gazebo(tmp_path):
    module = _load_module("persistent_voice_nav_base.launch.py")
    world = tmp_path / "generated.sdf"
    world.write_text("<sdf/>", encoding="utf-8")
    gazebo_server = object()

    success_actions = module._handle_world_xacro_exit(
        SimpleNamespace(returncode=0),
        None,
        generated_world=world,
        gazebo_server=gazebo_server,
    )
    failed_actions = module._handle_world_xacro_exit(
        SimpleNamespace(returncode=3),
        None,
        generated_world=world,
        gazebo_server=gazebo_server,
    )

    assert success_actions == [gazebo_server]
    assert gazebo_server not in failed_actions


def test_persistent_launches_never_create_session_authority_manager():
    for filename in (
        "persistent_voice_nav_base.launch.py",
        "persistent_mapping_stage.launch.py",
        "persistent_navigation_stage.launch.py",
    ):
        description = _load_description(filename)
        executables = {
            _literal_node_executable(entity)
            for entity in _contract_entities(description)
            if isinstance(entity, Node)
        }
        source = (LAUNCH_ROOT / filename).read_text(encoding="utf-8")

        assert "control_authority" not in executables
        assert "control_authority_manager_node" not in source
        assert "control_authority_manager_enabled" not in source


def test_persistent_launch_interface_is_explicitly_installed():
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    for filename in (
        "persistent_voice_nav_base.launch.py",
        "persistent_mapping_stage.launch.py",
        "persistent_navigation_stage.launch.py",
    ):
        assert f"launch/{filename}" in cmake
    assert "DESTINATION share/${PROJECT_NAME}/launch" in cmake
