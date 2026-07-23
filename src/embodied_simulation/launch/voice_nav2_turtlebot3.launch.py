import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

from embodied_agent_bringup.agent_launch_contract import (
    declare_forwarded_agent_arguments,
    forwarded_agent_configurations,
    forwarded_agent_launch_arguments,
)


def include_launch(package, filename, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), "launch", filename)
        ),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def as_python_bool(value):
    return PythonExpression(["'True' if '", value, "'.lower() == 'true' else 'False'"])


def default_gz_partition():
    """隔离 Nav2 自带 Gazebo server，防止旧世界令 /clock 改为 namespaced topic。"""

    configured = os.environ.get("GZ_PARTITION", "").strip()
    if configured:
        return configured
    domain_id = os.environ.get("ROS_DOMAIN_ID", "0")
    return f"embodied_agent_{domain_id}"


def validate_control_authority_owner(context):
    """禁止阶段 launch 私自创建缺少 quiescence coordinator 的 manager。"""

    authority_enabled = (
        LaunchConfiguration("control_authority_enabled")
        .perform(context)
        .strip()
        .lower()
        == "true"
    )
    child_owns_manager = (
        LaunchConfiguration("control_authority_manager_enabled")
        .perform(context)
        .strip()
        .lower()
        == "true"
    )
    if authority_enabled and child_owns_manager:
        raise RuntimeError(
            "control authority requires the session-level manager and "
            "quiescence coordinator; set control_authority_manager_enabled=false "
            "and start the manager from voice_slam_nav_showcase.sh auto"
        )
    return []


def generate_launch_description():
    simulation_share = get_package_share_directory("embodied_simulation")
    nav2_share = get_package_share_directory("nav2_bringup")
    nav2_params = os.path.join(nav2_share, "params", "nav2_params.yaml")
    nav2_map = os.path.join(simulation_share, "maps", "voice_demo.yaml")
    default_world = os.path.join(simulation_share, "worlds", "voice_demo.sdf.xacro")
    default_rviz = os.path.join(simulation_share, "rviz", "voice_nav2_demo.rviz")
    control_config = os.path.join(
        simulation_share, "config", "simulation_control.yaml"
    )
    default_velocity_mux_config = os.path.join(
        simulation_share, "config", "twist_mux.yaml"
    )

    launch_agent = LaunchConfiguration("launch_agent")
    agent_type = LaunchConfiguration("agent_type")
    provider_mode = LaunchConfiguration("provider_mode")
    agent_config = forwarded_agent_configurations()
    microphone = agent_config["microphone_enabled"]
    capture_enabled = LaunchConfiguration("capture_enabled")
    speaker_enabled = LaunchConfiguration("speaker_enabled")
    vad_provider = LaunchConfiguration("vad_provider")
    speech_start_threshold = LaunchConfiguration("speech_start_threshold")
    vad_speech_start_ms = LaunchConfiguration("vad_speech_start_ms")
    speech_end_silence_s = LaunchConfiguration("speech_end_silence_s")
    min_utterance_ms = LaunchConfiguration("min_utterance_ms")
    max_utterance_s = LaunchConfiguration("max_utterance_s")
    silero_model_path = LaunchConfiguration("silero_model_path")
    silero_use_onnx = LaunchConfiguration("silero_use_onnx")
    silero_threshold = LaunchConfiguration("silero_threshold")
    silero_end_threshold = LaunchConfiguration("silero_end_threshold")
    kws_provider = LaunchConfiguration("kws_provider")
    sherpa_tokens = LaunchConfiguration("sherpa_tokens")
    sherpa_encoder = LaunchConfiguration("sherpa_encoder")
    sherpa_decoder = LaunchConfiguration("sherpa_decoder")
    sherpa_joiner = LaunchConfiguration("sherpa_joiner")
    sherpa_keywords_file = LaunchConfiguration("sherpa_keywords_file")
    openwakeword_models = LaunchConfiguration("openwakeword_models")
    openwakeword_threshold = LaunchConfiguration("openwakeword_threshold")
    livekit_wakeword_models = LaunchConfiguration("livekit_wakeword_models")
    livekit_wakeword_threshold = LaunchConfiguration("livekit_wakeword_threshold")
    audio_enhancer = LaunchConfiguration("audio_enhancer")
    aec_enabled = LaunchConfiguration("aec_enabled")
    noise_suppression_enabled = LaunchConfiguration("noise_suppression_enabled")
    auto_gain_enabled = LaunchConfiguration("auto_gain_enabled")
    lifecycle_autostart = LaunchConfiguration("lifecycle_autostart")
    nav_action_timeout_s = LaunchConfiguration("nav_action_timeout_s")
    nav2_progress_radius = LaunchConfiguration("nav2_progress_radius")
    nav2_progress_timeout = LaunchConfiguration("nav2_progress_timeout")
    params_file = LaunchConfiguration("params_file")
    velocity_mux_config = LaunchConfiguration("velocity_mux_config")
    control_authority_enabled = LaunchConfiguration("control_authority_enabled")
    control_authority_manager_enabled = LaunchConfiguration(
        "control_authority_manager_enabled"
    )
    authority_lease_ms = LaunchConfiguration("authority_lease_ms")
    autonomy_velocity_timeout_ms = LaunchConfiguration(
        "autonomy_velocity_timeout_ms"
    )
    keyboard_velocity_timeout_ms = LaunchConfiguration(
        "keyboard_velocity_timeout_ms"
    )
    velocity_gate_publish_rate_hz = LaunchConfiguration(
        "velocity_gate_publish_rate_hz"
    )
    velocity_gate_max_linear_x = LaunchConfiguration(
        "velocity_gate_max_linear_x"
    )
    velocity_gate_max_angular_z = LaunchConfiguration(
        "velocity_gate_max_angular_z"
    )
    keyboard_authority_source = LaunchConfiguration(
        "keyboard_authority_source"
    )
    slam = LaunchConfiguration("slam")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    headless = LaunchConfiguration("headless")
    world = LaunchConfiguration("world")
    use_composition = LaunchConfiguration("use_composition")
    executor_plugin = LaunchConfiguration("executor_plugin")
    readiness_stale_timeout_s = LaunchConfiguration("readiness_stale_timeout_s")
    gz_partition = LaunchConfiguration("gz_partition")
    enable_dynamic_obstacle_layer = LaunchConfiguration(
        "enable_dynamic_obstacle_layer"
    )

    online_condition = IfCondition(
        PythonExpression([
            "'", launch_agent, "' == 'true' and '", agent_type, "' == 'online'"
        ])
    )
    offline_condition = IfCondition(
        PythonExpression([
            "'", launch_agent, "' == 'true' and '", agent_type, "' == 'offline'"
        ])
    )
    # 预测层只在显式启用时启动 tracker。普通语音/Nav2 演示保持官方参数，
    # 完整 SLAM 门禁则同时传入已插入 PredictedObstacleLayer 的 params_file。
    dynamic_obstacle_tracker = Node(
        package="embodied_navigation",
        executable="dynamic_obstacle_tracker_node",
        name="dynamic_obstacle_tracker",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("embodied_navigation"),
                "config",
                "navigation_overrides.yaml",
            ),
            {"use_sim_time": True},
        ],
        condition=IfCondition(enable_dynamic_obstacle_layer),
    )

    # 不修改 /opt/ros 中的 Nav2 默认文件，而是在 launch 上下文生成临时参数副本。
    # WSL/Gazebo 低实时率时 0.5m/10s 的默认进度门槛过于激进；项目级覆盖仍保留
    # SimpleProgressChecker 的安全约束，同时避免低速有效运动被误判为卡死。
    configured_nav2_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={
            "required_movement_radius": nav2_progress_radius,
            "movement_time_allowance": nav2_progress_timeout,
            "stop_on_failure": "true",
            # Collision Monitor 必须是唯一的底盘速度出口。mux 先完成多输入仲裁，
            # 再由 Collision Monitor 对选中速度执行最后一道雷达安全约束。
            "cmd_vel_in_topic": "/control/selected/cmd_vel",
            "cmd_vel_out_topic": "/cmd_vel",
        },
        convert_types=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument("launch_agent", default_value="true"),
        DeclareLaunchArgument("agent_type", default_value="online"),
        DeclareLaunchArgument("provider_mode", default_value="mock"),
        # Nav2 场景只覆盖端点等待和队列时长，其余默认值来自 Agent 参数 schema。
        *declare_forwarded_agent_arguments(default_overrides={
            "voice_session_timeout_s": 120.0,
            "continuous_command_max_age_s": 120.0,
            "asr_commit_delay_ms": 300,
        }),
        DeclareLaunchArgument("capture_enabled", default_value=microphone),
        DeclareLaunchArgument("speaker_enabled", default_value="false"),
        DeclareLaunchArgument("vad_provider", default_value="energy"),
        DeclareLaunchArgument("speech_start_threshold", default_value="0.018"),
        DeclareLaunchArgument("vad_speech_start_ms", default_value="96.0"),
        DeclareLaunchArgument("speech_end_silence_s", default_value="0.7"),
        DeclareLaunchArgument("min_utterance_ms", default_value="100.0"),
        DeclareLaunchArgument("max_utterance_s", default_value="12.0"),
        DeclareLaunchArgument("silero_model_path", default_value=""),
        DeclareLaunchArgument("silero_use_onnx", default_value="true"),
        DeclareLaunchArgument("silero_threshold", default_value="0.5"),
        DeclareLaunchArgument("silero_end_threshold", default_value="0.35"),
        DeclareLaunchArgument("kws_provider", default_value="none"),
        DeclareLaunchArgument("sherpa_tokens", default_value=""),
        DeclareLaunchArgument("sherpa_encoder", default_value=""),
        DeclareLaunchArgument("sherpa_decoder", default_value=""),
        DeclareLaunchArgument("sherpa_joiner", default_value=""),
        DeclareLaunchArgument("sherpa_keywords_file", default_value=""),
        DeclareLaunchArgument("openwakeword_models", default_value=""),
        DeclareLaunchArgument("openwakeword_threshold", default_value="0.5"),
        DeclareLaunchArgument("livekit_wakeword_models", default_value=""),
        DeclareLaunchArgument("livekit_wakeword_threshold", default_value="0.5"),
        DeclareLaunchArgument("audio_enhancer", default_value="nlms"),
        DeclareLaunchArgument("aec_enabled", default_value="true"),
        DeclareLaunchArgument("noise_suppression_enabled", default_value="false"),
        DeclareLaunchArgument("auto_gain_enabled", default_value="false"),
        DeclareLaunchArgument("lifecycle_autostart", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("slam", default_value="false"),
        DeclareLaunchArgument("map", default_value=nav2_map),
        DeclareLaunchArgument("params_file", default_value=nav2_params),
        DeclareLaunchArgument(
            "velocity_mux_config",
            default_value=default_velocity_mux_config,
            description="twist_mux priorities/timeouts for Nav2 and voice autonomy",
        ),
        # 默认 false 保持既有验收入口兼容；统一演示入口显式开启 typed 控制权。
        DeclareLaunchArgument("control_authority_enabled", default_value="false"),
        # 控制权 manager 必须由会话根与 quiescence coordinator 一起持有。
        # 该兼容参数保留用于给旧调用者明确报错，不能在阶段 launch 内设为 true。
        DeclareLaunchArgument(
            "control_authority_manager_enabled",
            default_value="false",
        ),
        DeclareLaunchArgument("authority_state_heartbeat_ms", default_value="200"),
        DeclareLaunchArgument("authority_lease_ms", default_value="750"),
        DeclareLaunchArgument("autonomy_velocity_timeout_ms", default_value="600"),
        DeclareLaunchArgument("keyboard_velocity_timeout_ms", default_value="600"),
        DeclareLaunchArgument("velocity_gate_publish_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("velocity_gate_max_linear_x", default_value="0.26"),
        DeclareLaunchArgument("velocity_gate_max_angular_z", default_value="1.82"),
        DeclareLaunchArgument(
            "keyboard_authority_source", default_value="keyboard_teleop"
        ),
        DeclareLaunchArgument("nav2_progress_radius", default_value="0.10"),
        DeclareLaunchArgument("nav2_progress_timeout", default_value="30.0"),
        DeclareLaunchArgument("rviz_config_file", default_value=default_rviz),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("use_composition", default_value="true"),
        DeclareLaunchArgument(
            "executor_plugin",
            default_value="embodied_simulation/Nav2RobotExecutor",
            description=(
                "mapping 阶段使用 GazeboRobotExecutor 做语音遥控，"
                "localization/navigation 阶段使用 Nav2RobotExecutor"
            ),
        ),
        DeclareLaunchArgument("nav_action_timeout_s", default_value="180.0"),
        # Gazebo、AMCL/Nav2 与 Agent 并行冷启动时会跨越数秒。健康事件是
        # transient-local 状态快照而非高频心跳，因此这里的窗口必须覆盖冷启动。
        DeclareLaunchArgument("readiness_stale_timeout_s", default_value="30.0"),
        DeclareLaunchArgument("gz_partition", default_value=default_gz_partition()),
        DeclareLaunchArgument(
            "enable_dynamic_obstacle_layer",
            default_value="false",
            description="Start typed dynamic tracker; params_file must contain the costmap plugin",
        ),
        DeclareLaunchArgument("x_pose", default_value="-2.0"),
        DeclareLaunchArgument("y_pose", default_value="-0.5"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        OpaqueFunction(function=validate_control_authority_owner),
        SetEnvironmentVariable("GZ_PARTITION", gz_partition),
        SetEnvironmentVariable("IGN_PARTITION", gz_partition),
        # Jazzy 的 velocity_smoother 输出名是固定的 cmd_vel_smoothed。这里把 remap
        # 限定在 Nav2 bringup 组内，避免全局 remap 误伤 Agent 或手动 executor。
        GroupAction(
            actions=[
                SetRemap(
                    src="cmd_vel_smoothed",
                    dst="/control/nav2/cmd_vel",
                ),
                # 复用 Nav2 官方 TurtleBot3 bringup 的成熟导航栈；默认 map/world/RViz
                # 指向本项目资产，让演示脚本、审计报告和简历讲解都有稳定的项目内入口。
                include_launch(
                    "nav2_bringup",
                    "tb3_simulation_launch.py",
                    {
                        # Nav2 官方 launch 内部使用 PythonExpression(['not ', use_composition])
                        # 这类表达式，必须收到 Python 可识别的 True/False；本项目对外仍保留
                        # ROS 常见的小写 true/false 参数，避免用户命令行习惯被打破。
                        "slam": as_python_bool(slam),
                        "map": LaunchConfiguration("map"),
                        "params_file": configured_nav2_params,
                        "rviz_config_file": rviz_config_file,
                        "use_rviz": as_python_bool(use_rviz),
                        "headless": as_python_bool(headless),
                        "world": world,
                        "autostart": "true",
                        "use_sim_time": "true",
                        "use_composition": as_python_bool(use_composition),
                        "x_pose": LaunchConfiguration("x_pose"),
                        "y_pose": LaunchConfiguration("y_pose"),
                        "yaw": LaunchConfiguration("yaw"),
                    },
                ),
            ],
        ),
        # 兼容模式：旧验收无需权限 manager，自治 mux 直接进入 Collision Monitor。
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux",
            output="screen",
            parameters=[
                velocity_mux_config,
                {"use_sim_time": True},
            ],
            remappings=[
                ("/cmd_vel_out", "/control/selected/cmd_vel"),
            ],
            condition=UnlessCondition(control_authority_enabled),
        ),
        # 控制权模式：mux 不再理解 HOLD/ESTOP，只生成一个统一的自治速度。
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux",
            output="screen",
            parameters=[
                velocity_mux_config,
                {"use_sim_time": True},
            ],
            remappings=[
                ("/cmd_vel_out", "/control/autonomy/cmd_vel"),
            ],
            condition=IfCondition(control_authority_enabled),
        ),
        Node(
            package="embodied_agent_cpp",
            executable="velocity_authority_gate",
            name="velocity_authority_gate",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "authority_lease_ms": ParameterValue(
                        authority_lease_ms, value_type=int
                    ),
                    "autonomy_timeout_ms": ParameterValue(
                        autonomy_velocity_timeout_ms, value_type=int
                    ),
                    "keyboard_timeout_ms": ParameterValue(
                        keyboard_velocity_timeout_ms, value_type=int
                    ),
                    "publish_rate_hz": ParameterValue(
                        velocity_gate_publish_rate_hz, value_type=float
                    ),
                    "max_abs_linear_x": ParameterValue(
                        velocity_gate_max_linear_x, value_type=float
                    ),
                    "max_abs_angular_z": ParameterValue(
                        velocity_gate_max_angular_z, value_type=float
                    ),
                    "expected_keyboard_source": keyboard_authority_source,
                }
            ],
            condition=IfCondition(control_authority_enabled),
        ),
        dynamic_obstacle_tracker,
        include_launch(
            "embodied_simulation",
            "simulation_control.launch.py",
            {
                "config": control_config,
                "use_sim_time": "true",
                "use_typed_actions": "true",
                "use_behavior_tree": "true",
                "executor_plugin": executor_plugin,
                "autostart": lifecycle_autostart,
                "action_timeout_s": nav_action_timeout_s,
                "cmd_vel_topic": "/control/voice/cmd_vel",
                "readiness_profile": "voice_nav2",
                "readiness_stale_timeout_s": readiness_stale_timeout_s,
                "readiness_required_components": (
                    "audio_frontend,agent,action_guard,"
                    "typed_action_bridge,simulation_control"
                ),
            },
        ),
        include_launch(
            "embodied_online_agent",
            "online_agent.launch.py",
            {
                **forwarded_agent_launch_arguments(provider_mode),
                "capture_enabled": capture_enabled,
                "speaker_enabled": speaker_enabled,
                "vad_provider": vad_provider,
                "speech_start_threshold": speech_start_threshold,
                "vad_speech_start_ms": vad_speech_start_ms,
                "speech_end_silence_s": speech_end_silence_s,
                "min_utterance_ms": min_utterance_ms,
                "max_utterance_s": max_utterance_s,
                "silero_model_path": silero_model_path,
                "silero_use_onnx": silero_use_onnx,
                "silero_threshold": silero_threshold,
                "silero_end_threshold": silero_end_threshold,
                "kws_provider": kws_provider,
                "sherpa_tokens": sherpa_tokens,
                "sherpa_encoder": sherpa_encoder,
                "sherpa_decoder": sherpa_decoder,
                "sherpa_joiner": sherpa_joiner,
                "sherpa_keywords_file": sherpa_keywords_file,
                "openwakeword_models": openwakeword_models,
                "openwakeword_threshold": openwakeword_threshold,
                "livekit_wakeword_models": livekit_wakeword_models,
                "livekit_wakeword_threshold": livekit_wakeword_threshold,
                "audio_enhancer": audio_enhancer,
                "aec_enabled": aec_enabled,
                "noise_suppression_enabled": noise_suppression_enabled,
                "auto_gain_enabled": auto_gain_enabled,
                "authority_gate_enabled": control_authority_enabled,
                "hardware_enabled": "false",
                "lifecycle_autostart": lifecycle_autostart,
            },
            online_condition,
        ),
        include_launch(
            "embodied_offline_agent",
            "offline_agent.launch.py",
            {
                **forwarded_agent_launch_arguments(provider_mode),
                "capture_enabled": capture_enabled,
                "speaker_enabled": speaker_enabled,
                "vad_provider": vad_provider,
                "speech_start_threshold": speech_start_threshold,
                "vad_speech_start_ms": vad_speech_start_ms,
                "speech_end_silence_s": speech_end_silence_s,
                "min_utterance_ms": min_utterance_ms,
                "max_utterance_s": max_utterance_s,
                "silero_model_path": silero_model_path,
                "silero_use_onnx": silero_use_onnx,
                "silero_threshold": silero_threshold,
                "silero_end_threshold": silero_end_threshold,
                "kws_provider": kws_provider,
                "sherpa_tokens": sherpa_tokens,
                "sherpa_encoder": sherpa_encoder,
                "sherpa_decoder": sherpa_decoder,
                "sherpa_joiner": sherpa_joiner,
                "sherpa_keywords_file": sherpa_keywords_file,
                "openwakeword_models": openwakeword_models,
                "openwakeword_threshold": openwakeword_threshold,
                "livekit_wakeword_models": livekit_wakeword_models,
                "livekit_wakeword_threshold": livekit_wakeword_threshold,
                "audio_enhancer": audio_enhancer,
                "aec_enabled": aec_enabled,
                "noise_suppression_enabled": noise_suppression_enabled,
                "auto_gain_enabled": auto_gain_enabled,
                "authority_gate_enabled": control_authority_enabled,
                "hardware_enabled": "false",
                "lifecycle_autostart": lifecycle_autostart,
            },
            offline_condition,
        ),
        LifecycleNode(
            package="embodied_agent_cpp",
            executable="action_guard",
            name="action_guard",
            namespace="",
            output="screen",
            parameters=[
                {
                    "authority_gate_enabled": ParameterValue(
                        control_authority_enabled, value_type=bool
                    )
                }
            ],
            condition=UnlessCondition(launch_agent),
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="action_guard_lifecycle_manager",
            output="screen",
            parameters=[{
                "autostart": ParameterValue(lifecycle_autostart, value_type=bool),
                "node_names": ["action_guard"],
                "bond_timeout": 0.0,
            }],
            condition=UnlessCondition(launch_agent),
        ),
    ])
