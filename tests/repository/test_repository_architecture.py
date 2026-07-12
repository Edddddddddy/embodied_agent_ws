"""仓库架构、强类型接口与生命周期边界约束。"""

from repository_test_support import BRINGUP_ROOT, CORE_ROOT, ROOT, VOICE_FRONTEND_ROOT


def test_repository_contracts_remain_split_by_architecture_topic():
    repository = ROOT / "tests" / "repository"
    assert not (repository / "test_repository_layout.py").exists()
    for name in (
        "test_repository_architecture.py",
        "test_repository_delivery.py",
        "test_repository_voice_runtime.py",
    ):
        path = repository / name
        assert path.is_file()
        assert len(path.read_text(encoding="utf-8").splitlines()) < 900
    assert (ROOT / "docs" / "ARCHITECTURE_AUDIT.md").is_file()


def test_agent_core_is_the_one_way_shared_dependency():
    """公共实现与输入 Adapter 必须独立，禁止 online/offline 相互依赖。"""

    core_package = ROOT / "src" / "embodied_agent_core"
    assert (core_package / "package.xml").is_file()
    assert (core_package / "setup.py").is_file()
    assert (core_package / "config" / "command_normalization_zh.yaml").is_file()
    assert (core_package / "prompts" / "system_prompt_zh.txt").is_file()
    voice_package = ROOT / "src" / "embodied_voice_frontend"
    assert (voice_package / "package.xml").is_file()
    assert (voice_package / "setup.py").is_file()

    core_source = "\n".join(
        path.read_text(encoding="utf-8") for path in CORE_ROOT.glob("*.py")
    )
    assert "embodied_online_agent" not in core_source
    assert "embodied_offline_agent" not in core_source

    for package_name, node_name in (
        ("embodied_online_agent", "online_agent_node.py"),
        ("embodied_offline_agent", "offline_agent_node.py"),
    ):
        package_root = ROOT / "src" / package_name
        manifest = (package_root / "package.xml").read_text(encoding="utf-8")
        node = (package_root / package_name / node_name).read_text(encoding="utf-8")
        assert "<exec_depend>embodied_agent_core</exec_depend>" in manifest
        assert "<exec_depend>embodied_voice_frontend</exec_depend>" in manifest
        other_agent = (
            "embodied_offline_agent"
            if package_name == "embodied_online_agent"
            else "embodied_online_agent"
        )
        assert f"<exec_depend>{other_agent}</exec_depend>" not in manifest
        assert "from embodied_agent_core.agent_control_plane import" in node
        assert "from embodied_agent_core.agent_lifecycle_runtime import" in node

    # 删除旧实现而非保留转发 shim，确保公共逻辑只有一个权威位置。
    old_online_root = (
        ROOT / "src" / "embodied_online_agent" / "embodied_online_agent"
    )
    for moved_module in (
        "agent_control_plane.py",
        "agent_lifecycle_runtime.py",
        "agent_ros_io.py",
        "continuous_voice.py",
        "ros_event_transport.py",
    ):
        assert not (old_online_root / moved_module).exists()
        assert (CORE_ROOT / moved_module).is_file()

    for adapter in (
        "keyword_wake_node.py",
        "silero_vad_node.py",
        "speaker_identity_node.py",
        "webrtc_vad_node.py",
    ):
        assert not (old_online_root / adapter).exists()
        assert (VOICE_FRONTEND_ROOT / adapter).is_file()


def test_bringup_owns_deployment_topology_without_polluting_domain_core():
    package = ROOT / "src" / "embodied_agent_bringup"
    assert (package / "package.xml").is_file()
    assert (package / "setup.py").is_file()
    for module in (
        "agent_launch_contract.py",
        "voice_frontend_launch_contract.py",
        "agent_deployment_launch_contract.py",
    ):
        assert (BRINGUP_ROOT / module).is_file()
        assert not (CORE_ROOT / module).exists()

    core_manifest = (
        ROOT / "src" / "embodied_agent_core" / "package.xml"
    ).read_text(encoding="utf-8")
    assert "<exec_depend>launch</exec_depend>" not in core_manifest
    assert "<exec_depend>launch_ros</exec_depend>" not in core_manifest

    for package_name in (
        "embodied_online_agent",
        "embodied_offline_agent",
        "embodied_simulation",
    ):
        manifest = (ROOT / "src" / package_name / "package.xml").read_text(
            encoding="utf-8"
        )
        assert "<exec_depend>embodied_agent_bringup</exec_depend>" in manifest


def test_online_and_offline_launch_share_voice_frontend_contract():
    """provider launch 只编排 Agent 特有能力，不复制前端节点和参数映射。"""
    contract = BRINGUP_ROOT / "voice_frontend_launch_contract.py"
    assert contract.is_file()
    contract_text = contract.read_text(encoding="utf-8")
    for executable in (
        'executable="speaker_identity"',
        'executable="audio_frontend"',
        'executable="webrtc_vad"',
        'executable="silero_vad"',
        'executable="keyword_wake"',
    ):
        assert executable in contract_text

    for package_name, launch_name in (
        ("embodied_online_agent", "online_agent.launch.py"),
        ("embodied_offline_agent", "offline_agent.launch.py"),
    ):
        launch = (
            ROOT / "src" / package_name / "launch" / launch_name
        ).read_text(encoding="utf-8")
        assert "declare_voice_frontend_arguments" in launch
        assert "voice_frontend_nodes" in launch
        assert 'executable="audio_frontend"' not in launch
        assert 'executable="silero_vad"' not in launch

def test_online_and_offline_launch_share_safe_deployment_contract():
    """ActionGuard、Lifecycle 顺序与硬件参数必须有一个权威实现。"""
    contract = BRINGUP_ROOT / "agent_deployment_launch_contract.py"
    assert contract.is_file()
    text = contract.read_text(encoding="utf-8")
    assert 'managed_nodes = ["action_guard", agent_name]' in text
    assert 'executable="hardware_controller"' in text
    assert "uart_baud_rate" in text
    assert "spi_speed_hz" in text

    for package_name, launch_name, agent_name in (
        ("embodied_online_agent", "online_agent.launch.py", "online_agent"),
        ("embodied_offline_agent", "offline_agent.launch.py", "offline_agent"),
    ):
        launch = (
            ROOT / "src" / package_name / "launch" / launch_name
        ).read_text(encoding="utf-8")
        assert "declare_agent_deployment_arguments" in launch
        assert f'agent_deployment_nodes("{agent_name}")' in launch
        assert 'executable="action_guard"' not in launch
        assert 'executable="hardware_controller"' not in launch

def test_robot_action_transport_is_fully_typed_without_legacy_json_adapter():
    """控制接口必须保持 typed；JSON 只能用于日志/报告，不能重新进入机器人链路。"""

    interfaces = ROOT / "src" / "embodied_agent_interfaces" / "msg"
    for name in (
        "RobotCommand.msg",
        "RobotCommandFeedback.msg",
        "RobotCommandResult.msg",
    ):
        assert (interfaces / name).is_file()

    cpp = ROOT / "src" / "embodied_agent_cpp"
    guard = (cpp / "src" / "action_guard_node.cpp").read_text(encoding="utf-8")
    bridge = (cpp / "src" / "typed_action_bridge_node.cpp").read_text(
        encoding="utf-8"
    )
    sequencer = (CORE_ROOT / "action_sequence.py").read_text(encoding="utf-8")

    assert "Subscription<\n    embodied_agent_interfaces::msg::RobotCommand>" in guard
    assert "RobotCommandFeedback" in bridge
    assert "RobotCommandResult" in bridge
    assert "ActionScheduler" in bridge
    assert '"/diagnostics"' in bridge
    assert "async_cancel_goal" in bridge
    assert "nlohmann/json" not in bridge
    assert "json.loads" not in sequencer
    assert "_legacy_results" not in sequencer
    assert not (cpp / "src" / "robot_command_adapter.cpp").exists()
    assert not (cpp / "include" / "embodied_agent_cpp" / "robot_command_adapter.hpp").exists()
    assert (cpp / "include" / "embodied_agent_cpp" / "action_scheduler.hpp").is_file()
    assert (cpp / "src" / "action_scheduler.cpp").is_file()
    assert (cpp / "test" / "test_action_scheduler.cpp").is_file()

    command_msg = (interfaces / "RobotCommand.msg").read_text(encoding="utf-8")
    assert "bool priority" in command_msg
    acceptance = (ROOT / "scripts" / "acceptance_test.sh").read_text(encoding="utf-8")
    assert "cpp-action-scheduler" in acceptance

def test_agent_parameter_contract_has_one_authoritative_schema():
    """在线/离线节点不得重新复制默认参数表，组合 launch 也必须复用转发契约。"""

    online_package = ROOT / "src" / "embodied_online_agent"
    schema = CORE_ROOT / "agent_parameters.py"
    launch_contract = BRINGUP_ROOT / "agent_launch_contract.py"
    assert schema.is_file()
    assert launch_contract.is_file()

    for node_path in (
        online_package / "embodied_online_agent" / "online_agent_node.py",
        ROOT
        / "src"
        / "embodied_offline_agent"
        / "embodied_offline_agent"
        / "offline_agent_node.py",
    ):
        text = node_path.read_text(encoding="utf-8")
        assert "declare_agent_parameters" in text
        assert "self._agent_parameters =" in text
        assert "self._parameters = declare_agent_parameters" not in text
        assert "def _declare_parameters" not in text

    for launch_path in (
        ROOT / "src" / "embodied_simulation" / "launch" / "voice_turtlebot3.launch.py",
        ROOT
        / "src"
        / "embodied_simulation"
        / "launch"
        / "voice_nav2_turtlebot3.launch.py",
    ):
        text = launch_path.read_text(encoding="utf-8")
        assert "declare_forwarded_agent_arguments" in text
        assert "forwarded_agent_launch_arguments" in text
        assert 'DeclareLaunchArgument("continuous_command_queue_size"' not in text

def test_provider_yaml_does_not_duplicate_control_plane_defaults():
    """YAML 只描述 provider；公共控制面默认值由 agent_parameters.py 管理。"""

    forbidden = (
        "continuous_command_queue_size:",
        "voice_session_timeout_s:",
        "command_normalization_fuzzy_threshold:",
        "memory_max_turns:",
    )
    for profile in (
        ROOT / "src" / "embodied_online_agent" / "config" / "online_agent.yaml",
        ROOT / "src" / "embodied_offline_agent" / "config" / "offline_agent.yaml",
    ):
        text = profile.read_text(encoding="utf-8")
        for key in forbidden:
            assert key not in text

def test_parameter_smoke_uses_bounded_rclpy_client_instead_of_ros2cli_daemon():
    probe = ROOT / "scripts" / "ros_parameter_check.py"
    smoke = ROOT / "scripts" / "smoke_test_online_wake_config.sh"
    assert probe.is_file()
    probe_text = probe.read_text(encoding="utf-8")
    smoke_text = smoke.read_text(encoding="utf-8")
    assert "AsyncParameterClient" in probe_text
    assert "spin_until_future_complete" in probe_text
    assert "--timeout 15" in smoke_text
    assert "ros2 param get" not in smoke_text

def test_nav2_executor_failure_details_are_preserved():
    """Nav2 失败原因必须从 executor 透传到本项目 typed action。

    语音目标点导航现场排障最怕只看到 `blocked` 或 `executor_rejected`。这个结构测试锁住
    external detail seam：Nav2 插件负责记录 server unavailable、goal rejected、missed
    waypoints 等细节，simulation_control_node 负责把细节写进 feedback/result。
    """

    executor_header = (
        ROOT
        / "src"
        / "embodied_simulation"
        / "include"
        / "embodied_simulation"
        / "robot_executor.hpp"
    ).read_text(encoding="utf-8")
    executor_plugin = (
        ROOT
        / "src"
        / "embodied_simulation"
        / "src"
        / "nav2_robot_executor.cpp"
    ).read_text(encoding="utf-8")
    control_node = (
        ROOT
        / "src"
        / "embodied_simulation"
        / "src"
        / "simulation_control_node.cpp"
    ).read_text(encoding="utf-8")
    action_runtime = (
        ROOT
        / "src"
        / "embodied_simulation"
        / "src"
        / "active_action_runtime.cpp"
    ).read_text(encoding="utf-8")

    assert "external_action_detail()" in executor_header
    for detail_token in (
        "server_unavailable",
        "goal_rejected",
        "goal_response_timeout_or_rejected",
        "missed_waypoints",
        "error_msg",
        "nav2:cancel_requested",
    ):
        assert detail_token in executor_plugin
    assert "executor_->external_action_detail()" in control_node
    assert "detail.empty() ? \"executor_rejected\" : detail" in control_node
    assert "Nav2 这类外部 action 的失败原因" in action_runtime

def test_robot_executor_backends_have_independent_implementation_units():
    """简单后端不能被迫携带 Nav2 Action client、线程和地图加载依赖。"""
    package = ROOT / "src" / "embodied_simulation"
    source_root = package / "src"
    cmake = (package / "CMakeLists.txt").read_text(encoding="utf-8")
    sources = {
        "GazeboRobotExecutor": source_root / "gazebo_robot_executor.cpp",
        "MockRobotExecutor": source_root / "mock_robot_executor.cpp",
        "Nav2RobotExecutor": source_root / "nav2_robot_executor.cpp",
    }

    assert not (source_root / "robot_executor_plugins.cpp").exists()
    for class_name, source in sources.items():
        text = source.read_text(encoding="utf-8")
        assert class_name in text
        assert "PLUGINLIB_EXPORT_CLASS" in text
        assert source.name in cmake

    for source in (sources["GazeboRobotExecutor"], sources["MockRobotExecutor"]):
        text = source.read_text(encoding="utf-8")
        assert "nav2_msgs" not in text
        assert "SingleThreadedExecutor" not in text
        assert "std::thread" not in text

def test_cpp_typed_action_demo_client_remains_available():
    """ROS2/C++ 求职展示必须保留独立 rclcpp_action client 示例。"""

    cmake = (ROOT / "src" / "embodied_agent_cpp" / "CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    demo_client_path = (
        ROOT
        / "src"
        / "embodied_agent_cpp"
        / "src"
        / "typed_action_demo_client.cpp"
    )
    demo_client = demo_client_path.read_text(encoding="utf-8")
    smoke_script = ROOT / "scripts" / "smoke_test_cpp_action_client.sh"
    acceptance = (ROOT / "scripts" / "acceptance_test.sh").read_text(
        encoding="utf-8"
    )
    learning = (ROOT / "docs" / "LEARNING_NOTES.md").read_text(encoding="utf-8")
    presentation = (ROOT / "docs" / "PROJECT_PRESENTATION_15MIN.md").read_text(
        encoding="utf-8"
    )

    assert demo_client_path.is_file()
    assert smoke_script.is_file()
    assert "add_executable(typed_action_demo_client" in cmake
    assert "typed_action_demo_client" in cmake
    assert "rclcpp_action::create_client<ExecuteRobotCommand>" in demo_client
    assert "feedback_callback" in demo_client
    assert "result_callback" in demo_client
    assert "cpp-action-client" in acceptance
    assert "typed_action_demo_client" in smoke_script.read_text(encoding="utf-8")
    assert "typed_action_demo_client.cpp" in learning
    assert "typed_action_demo_client.cpp" in presentation

def test_voice_control_events_are_strongly_typed_and_use_named_qos():
    """语音控制面属于中间件契约，禁止退回 String + JSON。"""

    interfaces = ROOT / "src" / "embodied_agent_interfaces" / "msg"
    online = (
        ROOT
        / "src"
        / "embodied_online_agent"
        / "embodied_online_agent"
        / "online_agent_node.py"
    ).read_text(encoding="utf-8")
    offline = (
        ROOT
        / "src"
        / "embodied_offline_agent"
        / "embodied_offline_agent"
        / "offline_agent_node.py"
    ).read_text(encoding="utf-8")
    transport = (CORE_ROOT / "ros_event_transport.py").read_text(encoding="utf-8")
    qos = (CORE_ROOT / "ros_qos.py").read_text(encoding="utf-8")
    event_adapter = (CORE_ROOT / "ros_agent_events.py").read_text(encoding="utf-8")
    ros_io = (CORE_ROOT / "agent_ros_io.py").read_text(encoding="utf-8")
    topics = (CORE_ROOT / "ros_topics.py").read_text(encoding="utf-8")

    assert (interfaces / "CommandContext.msg").is_file()
    assert (interfaces / "CommandQueueEvent.msg").is_file()
    assert (interfaces / "CommandExecutionEvent.msg").is_file()
    assert (interfaces / "WakeEvent.msg").is_file()
    assert (interfaces / "RecognitionFeedback.msg").is_file()
    assert (interfaces / "TextRewrite.msg").is_file()
    assert (interfaces / "CommandSlot.msg").is_file()
    assert (interfaces / "NluCommand.msg").is_file()
    assert (interfaces / "NluParseEvent.msg").is_file()
    for node in (online, offline):
        assert "AgentRosIo" in node
        assert 'String, "/agent/command_queue"' not in node
        assert 'String, "/agent/command_execution"' not in node
        assert 'String, "/agent/wake_event"' not in node
        assert 'String, "/agent/recognition_feedback"' not in node
    for field, topic in (
        ("command_queue", "/agent/command_queue"),
        ("command_execution", "/agent/command_execution"),
        ("wake_event", "/agent/wake_event"),
        ("recognition_feedback", "/agent/recognition_feedback"),
        ("nlu_parse", "/agent/nlu_parse"),
    ):
        assert f"self._topics.{field}" in event_adapter
        assert topic in topics
    assert "event_qos()" in event_adapter
    assert "state_qos()" in event_adapter
    assert "RosAgentEventPublisher" in ros_io
    assert "audio_qos" in ros_io
    assert "event_qos" in ros_io
    assert "unsupported queue event" in transport
    assert "unsupported wake event" in transport
    assert "unsupported recognition feedback" in transport
    assert "nlu_parse_to_message" in transport
    assert "ReliabilityPolicy.RELIABLE" in qos
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in qos

    # 节点只能依赖共享 I/O Facade；硬编码接口名必须收敛到 Topic contract。
    for node in (online, offline):
        assert '"/agent/' not in node
        assert '"/audio/' not in node
        assert '"/robot/' not in node

def test_runtime_status_topics_are_strongly_typed():
    """运行状态也是跨进程契约，不能退回各节点自行拼装 JSON。"""
    interfaces = ROOT / "src" / "embodied_agent_interfaces" / "msg"
    for name in (
        "AudioFrontendStatus.msg",
        "VadEvent.msg",
        "KwsEvent.msg",
        "KwsScore.msg",
        "SimulationState.msg",
        "RobotActionAck.msg",
        "BehaviorTreeStatus.msg",
        "ComponentHealth.msg",
        "SystemReadiness.msg",
    ):
        assert (interfaces / name).is_file()

    online = ROOT / "src" / "embodied_online_agent" / "embodied_online_agent"
    runtime_transport = (CORE_ROOT / "runtime_status_transport.py").read_text(
        encoding="utf-8"
    )
    assert "action_ack_to_dict" in runtime_transport
    assert "simulation_state_to_dict" in runtime_transport
    assert not (CORE_ROOT / "wake_event_input.py").exists()

    sources = [
        VOICE_FRONTEND_ROOT / "keyword_wake_node.py",
        VOICE_FRONTEND_ROOT / "silero_vad_node.py",
        VOICE_FRONTEND_ROOT / "webrtc_vad_node.py",
        ROOT / "src" / "embodied_agent_cpp" / "src" / "audio_frontend_node.cpp",
        ROOT / "src" / "embodied_simulation" / "src" / "simulation_control_node.cpp",
        ROOT / "src" / "embodied_simulation" / "src" / "simulation_ros_io.cpp",
    ]
    combined = "\n".join(path.read_text(encoding="utf-8") for path in sources)
    for topic in (
        "/agent/kws_event",
        "/agent/kws_score",
        "/audio/vad_event",
        "/audio/frontend_metrics",
        "robot/action_ack",
        "robot/simulation_state",
        "robot/bt_status",
    ):
        assert topic in combined
    assert 'create_publisher(String, "/audio/vad_event"' not in combined
    assert 'create_publisher(String, "/agent/kws_event"' not in combined

def test_speaker_memory_has_one_deep_module_and_typed_transport():
    interfaces = ROOT / "src" / "embodied_agent_interfaces" / "msg"
    for name in (
        "SpeakerIdentity.msg",
        "SpeakerEnrollRequest.msg",
        "SpeakerEnrollStatus.msg",
    ):
        assert (interfaces / name).is_file()
    online_root = ROOT / "src" / "embodied_online_agent" / "embodied_online_agent"
    service = (CORE_ROOT / "memory_command_service.py").read_text(encoding="utf-8")
    context_runtime = (CORE_ROOT / "user_context_runtime.py").read_text(
        encoding="utf-8"
    )
    transport = (CORE_ROOT / "speaker_transport.py").read_text(encoding="utf-8")
    online = (online_root / "online_agent_node.py").read_text(encoding="utf-8")
    offline = (
        ROOT
        / "src"
        / "embodied_offline_agent"
        / "embodied_offline_agent"
        / "offline_agent_node.py"
    ).read_text(encoding="utf-8")
    sidecar = (VOICE_FRONTEND_ROOT / "speaker_identity_node.py").read_text(
        encoding="utf-8"
    )

    assert "class MemoryCommandService" in service
    assert "def handle(" in service
    assert "class UserContextRuntime" in context_runtime
    assert "class UserContextSnapshot" in context_runtime
    for node in (online, offline):
        assert "UserContextRuntime" in node
        assert "MemoryCommandService" not in node
        assert "parse_memory_command" not in node
        assert "def _record_user_interaction" not in node
        assert "def _current_user_preferences" not in node
        assert "def _system_prompt_with_user_memory" not in node
        assert "def _actions_from_context" not in node
        assert 'String, "/agent/speaker_identity"' not in node
        assert 'String, "/agent/speaker_enroll_request"' not in node
    assert "identity_message_to_domain" in transport
    assert "SpeakerEnrollStatus" in sidecar
    assert "json.loads(message.data)" not in sidecar

def test_simulation_action_lifecycle_has_one_runtime_owner():
    """长动作状态应由深模块统一拥有，Lifecycle 节点只做 ROS 装配。"""
    package = ROOT / "src" / "embodied_simulation"
    header = package / "include" / "embodied_simulation" / "active_action_runtime.hpp"
    source = package / "src" / "active_action_runtime.cpp"
    node = (package / "src" / "simulation_control_node.cpp").read_text(
        encoding="utf-8"
    )

    assert header.is_file()
    assert source.is_file()
    assert "ActiveActionRuntime" in node
    assert "action_runtime_->update" in node
    assert "std::optional<ActionExecution> action_execution_" not in node
    assert "active_action_uses_external_result_" not in node

def test_simulation_ros_io_owns_managed_publishers_and_status_mapping():
    """仿真节点只编排控制流程，publisher 生命周期和状态映射必须集中管理。"""
    package = ROOT / "src" / "embodied_simulation"
    header = package / "include" / "embodied_simulation" / "simulation_ros_io.hpp"
    source = package / "src" / "simulation_ros_io.cpp"
    node_path = package / "src" / "simulation_control_node.cpp"
    cmake = (package / "CMakeLists.txt").read_text(encoding="utf-8")
    node = node_path.read_text(encoding="utf-8")
    ros_io = source.read_text(encoding="utf-8")

    assert header.is_file()
    assert source.is_file()
    assert "src/simulation_ros_io.cpp" in cmake
    assert "SimulationRosIo" in node
    for lifecycle_step in ("configure", "activate", "deactivate", "reset"):
        assert f"ros_io_->{lifecycle_step}" in node

    # 防止职责回流：新增状态 topic 时应扩展 SimulationRosIo，而不是膨胀节点。
    assert "create_publisher<" not in node
    assert "LifecyclePublisher<" not in node
    assert "action_sequence_" not in node
    assert "last_bt_status_" not in node
    assert len(node.splitlines()) < 800

    for profile in ("command_qos", "event_qos", "state_qos", "diagnostics_qos"):
        assert profile in ros_io
    assert "action_sequence_" in ros_io
    assert "last_bt_status_" in ros_io
    assert "on_activate" in ros_io
    assert "on_deactivate" in ros_io

def test_action_guard_buffers_only_the_dds_startup_window():
    """控制命令不能因 discovery 竞态丢失，也不能以 transient-local 重放旧动作。"""
    package = ROOT / "src" / "embodied_agent_cpp"
    header = package / "include" / "embodied_agent_cpp" / "guarded_command_outbox.hpp"
    node = (package / "src" / "action_guard_node.cpp").read_text(encoding="utf-8")

    assert header.is_file()
    assert "GuardedCommandOutbox" in node
    assert "downstream_wait_timeout_s" in node
    assert "get_subscription_count() > 0" in node
    assert "transient_local" not in node.lower()

def test_python_and_cpp_nodes_share_one_qos_vocabulary():
    middleware = ROOT / "src" / "embodied_agent_middleware"
    qos_header = (
        middleware / "include" / "embodied_agent_middleware" / "qos_profiles.hpp"
    )
    qos_text = qos_header.read_text(encoding="utf-8")
    python_qos = (CORE_ROOT / "ros_qos.py").read_text(encoding="utf-8")
    for profile in (
        "command_qos",
        "event_qos",
        "state_qos",
        "sensor_qos",
        "audio_qos",
        "diagnostics_qos",
    ):
        assert profile in qos_text
        assert f"def {profile}(" in python_qos

    sources = (
        ROOT / "src" / "embodied_agent_cpp" / "src" / "action_guard_node.cpp",
        ROOT / "src" / "embodied_agent_cpp" / "src" / "typed_action_bridge_node.cpp",
        ROOT / "src" / "embodied_agent_cpp" / "src" / "audio_frontend_node.cpp",
        ROOT / "src" / "embodied_simulation" / "src" / "simulation_ros_io.cpp",
    )
    for source in sources:
        text = source.read_text(encoding="utf-8")
        assert "embodied_agent_middleware/qos_profiles.hpp" in text
    assert "rclcpp::QoS(10)" not in sources[-1].read_text(encoding="utf-8")

    # Voice Adapter 只能选择项目语义，禁止重新散落 History/Reliability 魔法配置。
    for source in VOICE_FRONTEND_ROOT.glob("*_node.py"):
        text = source.read_text(encoding="utf-8")
        assert "QoSProfile" not in text
        assert "HistoryPolicy" not in text
        assert "ReliabilityPolicy" not in text
        assert "from embodied_agent_core.ros_qos import" in text

def test_system_readiness_is_typed_profile_based_and_heartbeat_driven():
    middleware = ROOT / "src" / "embodied_agent_middleware"
    registry = (
        middleware / "include" / "embodied_agent_middleware" / "component_health_registry.hpp"
    )
    aggregator = middleware / "src" / "system_readiness_node.cpp"
    assert registry.is_file()
    text = aggregator.read_text(encoding="utf-8")
    assert "required_components_csv" in text
    assert "stale_timeout_s" in text
    assert "SystemReadiness" in text

    launch = (
        ROOT / "src" / "embodied_simulation" / "launch" / "simulation_control.launch.py"
    ).read_text(encoding="utf-8")
    assert "system_readiness_node" in launch
    assert "readiness_required_components" in launch

    check = (ROOT / "scripts" / "system_readiness_check.py").read_text(
        encoding="utf-8"
    )
    assert '"/system/readiness"' in check

def test_online_and_offline_agents_have_real_lifecycle_resource_ownership():
    online_root = ROOT / "src" / "embodied_online_agent"
    offline_root = ROOT / "src" / "embodied_offline_agent"
    online_node = (
        online_root / "embodied_online_agent" / "online_agent_node.py"
    ).read_text(encoding="utf-8")
    offline_node = (
        offline_root / "embodied_offline_agent" / "offline_agent_node.py"
    ).read_text(encoding="utf-8")
    ros_io = (CORE_ROOT / "agent_ros_io.py").read_text(encoding="utf-8")
    lifecycle_runtime = (CORE_ROOT / "agent_lifecycle_runtime.py").read_text(
        encoding="utf-8"
    )

    for class_name, node in (
        ("OnlineAgentNode", online_node),
        ("OfflineAgentNode", offline_node),
    ):
        assert f"class {class_name}(LifecycleNode)" in node
        for callback in (
            "on_configure",
            "on_activate",
            "on_deactivate",
            "on_cleanup",
            "on_shutdown",
            "on_error",
        ):
            assert f"def {callback}(" in node
        assert "AgentRosIo" in node
        assert "AgentLifecycleRuntime" in node
        assert "self._runtime.activate(" in node
        assert "self._runtime.deactivate(" in node
        assert "self._runtime.release(" in node
        assert "self._lifecycle_active =" not in node
        assert "self._execution =" not in node
        assert "self._asr_endpoint =" not in node
        assert "start_background_turn(" in node
        assert "threading.Thread(\n            target=self._run_turn" not in node

    assert "create_lifecycle_publisher" in ros_io
    assert "create_subscription" in ros_io
    for ordered_step in (
        'cancel("lifecycle_deactivated")',
        "command_queue.clear()",
        "_publish_priority_stop()",
        "stop_input()",
        "_execution.stop(",
        'publish_stopped("lifecycle_inactive")',
    ):
        assert ordered_step in lifecycle_runtime
    assert (
        ROOT / "src" / "embodied_agent_core" / "test" / "test_agent_lifecycle_runtime.py"
    ).is_file()

    online_launch = (online_root / "launch" / "online_agent.launch.py").read_text(
        encoding="utf-8"
    )
    offline_launch = (
        offline_root / "launch" / "offline_agent.launch.py"
    ).read_text(encoding="utf-8")
    deployment_contract = (
        BRINGUP_ROOT / "agent_deployment_launch_contract.py"
    ).read_text(encoding="utf-8")
    assert 'managed_nodes = ["action_guard", agent_name]' in deployment_contract
    assert 'agent_deployment_nodes("online_agent")' in online_launch
    assert 'agent_deployment_nodes("offline_agent")' in offline_launch
    for launch in (online_launch, offline_launch):
        assert '"agent_lifecycle_autostart": False' in launch

    acceptance = (ROOT / "scripts" / "acceptance_test.sh").read_text(
        encoding="utf-8"
    )
    assert "agent-lifecycle" in acceptance
    assert (ROOT / "scripts" / "smoke_test_agent_lifecycle.sh").is_file()
    assert (ROOT / "tests" / "integration" / "test_agent_lifecycle.py").is_file()

def test_agent_turn_metrics_use_one_strongly_typed_ros_contract():
    """在线/离线指标必须共享 schema，禁止重新引入双 topic 或 JSON wire。"""

    interface = (
        ROOT
        / "src"
        / "embodied_agent_interfaces"
        / "msg"
        / "AgentTurnMetrics.msg"
    )
    ros_io = (CORE_ROOT / "agent_ros_io.py").read_text(encoding="utf-8")
    transport = (CORE_ROOT / "metrics_transport.py").read_text(encoding="utf-8")
    online = (
        ROOT
        / "src"
        / "embodied_online_agent"
        / "embodied_online_agent"
        / "online_agent_node.py"
    ).read_text(encoding="utf-8")
    offline = (
        ROOT
        / "src"
        / "embodied_offline_agent"
        / "embodied_offline_agent"
        / "offline_agent_node.py"
    ).read_text(encoding="utf-8")
    live_check = (ROOT / "scripts" / "continuous_live_check.py").read_text(
        encoding="utf-8"
    )

    assert interface.is_file()
    schema = interface.read_text(encoding="utf-8")
    assert "TARGET_UNKNOWN" in schema
    assert "llm_decode_tokens_per_s" in schema
    assert "tts_first_text_to_first_audio_ms" in schema
    assert "AgentTurnMetrics" in ros_io
    assert "String(data=payload)" not in ros_io
    assert "agent_turn_metrics_to_message" in online
    assert "agent_turn_metrics_to_message" in offline
    assert "agent_turn_metrics_message_to_dict" in live_check
    assert '"/agent/metrics"' in live_check
    assert "diagnostics_qos(depth=10)" in live_check
    assert "_finite_or_nan" in transport

    # 显式扫描保证新增脚本也受守卫约束；拆开 legacy 字符串，避免测试命中自身。
    legacy_metrics_topic = "/offline_agent" + "/metrics"
    tracked = "\n".join(
        path.read_text(encoding="utf-8", errors="ignore")
        for root in (
            ROOT / "src" / "embodied_online_agent",
            ROOT / "src" / "embodied_offline_agent",
            ROOT / "scripts",
            ROOT / "tests",
            ROOT / "docs",
        )
        for path in root.rglob("*")
        if path.is_file() and path.suffix in {".py", ".sh", ".md"}
    )
    assert legacy_metrics_topic not in tracked
