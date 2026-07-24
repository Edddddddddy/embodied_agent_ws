"""持久语音 SLAM/Nav2 演示的常驻运行时.

该 launch 的进程不会随 mapping/navigation stage 切换而重启。阶段特有的
SLAM/localization provider 与 RobotExecutor 由独立 stage launch 持有。
"""

import os
import tempfile
from functools import partial
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from embodied_agent_bringup.agent_launch_contract import (
    declare_forwarded_agent_arguments,
    forwarded_agent_configurations,
    forwarded_agent_launch_arguments,
)
from embodied_agent_bringup.voice_frontend_launch_contract import (
    declare_voice_frontend_arguments,
    voice_frontend_configurations,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def _include_launch(package: str, filename: str, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), "launch", filename)
        ),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def _as_python_bool(value):
    """把 ROS 小写布尔值转换为 Nav2 PythonExpression 可安全求值的字面量。"""

    return PythonExpression(
        ["'True' if '", value, "'.lower() == 'true' else 'False'"]
    )


def _default_gz_partition() -> str:
    configured = os.environ.get("GZ_PARTITION", "").strip()
    if configured:
        return configured
    return f"embodied_persistent_{os.environ.get('ROS_DOMAIN_ID', '0')}"


def _remove_generated_world(_context, path: str):
    """只清理由本 launch 生成的临时 SDF，不触碰用户传入的 xacro 资产."""
    if os.path.exists(path):
        os.remove(path)
    return []


def _handle_world_xacro_exit(
    event,
    _context,
    *,
    generated_world: Path,
    gazebo_server,
):
    """只有 xacro 成功且产物非空时才允许启动 Gazebo server."""

    if event.returncode == 0 and generated_world.is_file():
        if generated_world.stat().st_size > 0:
            return [gazebo_server]
    detail = (
        f"world xacro failed (returncode={event.returncode}, "
        f"output={generated_world})"
    )
    # fail-fast 比继续等 readiness 超时更容易现场诊断；Shutdown 仍会触发
    # OnShutdown 清理临时 SDF。
    return [EmitEvent(event=Shutdown(reason=detail))]


def generate_launch_description():
    simulation_share = get_package_share_directory("embodied_simulation")
    nav2_share = get_package_share_directory("nav2_bringup")
    simulation_model_share = get_package_share_directory("nav2_minimal_tb3_sim")
    # mkstemp 原子占位，避免 mktemp 的名称竞争；launch 正常/失败退出时统一清理。
    generated_world_fd, generated_world_name = tempfile.mkstemp(
        prefix="embodied_persistent_",
        suffix=".sdf",
    )
    os.close(generated_world_fd)
    generated_world = Path(generated_world_name)
    robot_urdf = os.path.join(
        simulation_model_share,
        "urdf",
        "turtlebot3_waffle.urdf",
    )
    with open(robot_urdf, encoding="utf-8") as stream:
        robot_description = stream.read()

    launch_agent = LaunchConfiguration("launch_agent")
    agent_type = LaunchConfiguration("agent_type")
    provider_mode = LaunchConfiguration("provider_mode")
    agent_config = forwarded_agent_configurations()
    voice_frontend_config = voice_frontend_configurations()
    microphone_enabled = agent_config["microphone_enabled"]
    use_rviz = LaunchConfiguration("use_rviz")
    headless = LaunchConfiguration("headless")
    control_authority_enabled = LaunchConfiguration(
        "control_authority_enabled"
    )
    lifecycle_autostart = LaunchConfiguration("lifecycle_autostart")
    gz_partition = LaunchConfiguration("gz_partition")
    params_file = LaunchConfiguration("params_file")
    velocity_mux_config = LaunchConfiguration("velocity_mux_config")
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
    persistent_zero_heartbeat_timeout_s = LaunchConfiguration(
        "persistent_zero_heartbeat_timeout_s"
    )
    configured_nav2_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={
            "required_movement_radius": LaunchConfiguration(
                "nav2_progress_radius"
            ),
            "movement_time_allowance": LaunchConfiguration(
                "nav2_progress_timeout"
            ),
            "stop_on_failure": "true",
            # Collision Monitor 是持久会话中唯一允许写入底盘 /cmd_vel 的节点。
            "cmd_vel_in_topic": "/control/selected/cmd_vel",
            "cmd_vel_out_topic": "/cmd_vel",
            # Nav2 默认在停车约 2 秒后抑制重复零速。对长时间演示，这会使
            # 后续 typed STOP 无法在最终控制边界形成“本次停车”的新鲜证据。
            # 延长的只是零速心跳窗口；障碍检测和非零速度裁决仍全部保留。
            "stop_pub_timeout": persistent_zero_heartbeat_timeout_s,
        },
        convert_types=True,
    )
    online_condition = IfCondition(
        PythonExpression(
            [
                "'",
                launch_agent,
                "' == 'true' and '",
                agent_type,
                "' == 'online'",
            ]
        )
    )
    offline_condition = IfCondition(
        PythonExpression(
            [
                "'",
                launch_agent,
                "' == 'true' and '",
                agent_type,
                "' == 'offline'",
            ]
        )
    )
    world_xacro = ExecuteProcess(
        cmd=[
            "xacro",
            "-o",
            str(generated_world),
            ["headless:=", headless],
            LaunchConfiguration("world"),
        ],
        output="screen",
    )
    gazebo_server = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", generated_world],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_agent", default_value="true"),
            DeclareLaunchArgument("agent_type", default_value="offline"),
            DeclareLaunchArgument("provider_mode", default_value="mock"),
            *declare_forwarded_agent_arguments(
                default_overrides={
                    "voice_session_timeout_s": 120.0,
                    "continuous_command_max_age_s": 120.0,
                    "asr_commit_delay_ms": 300,
                }
            ),
            # 现场音频校准参数是 base 的稳定 interface；统一复用 Agent
            # launch contract，防止 online/offline 两套参数继续漂移。
            *declare_voice_frontend_arguments(microphone_enabled),
            DeclareLaunchArgument(
                "asr_hotwords_score",
                default_value="3.0",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(
                    simulation_share,
                    "worlds",
                    "showcase_apartment.sdf.xacro",
                ),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    nav2_share,
                    "params",
                    "nav2_params.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "control_authority_enabled",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "rviz_config_file",
                default_value=os.path.join(
                    simulation_share,
                    "rviz",
                    "voice_nav2_demo.rviz",
                ),
            ),
            DeclareLaunchArgument(
                "robot_name",
                default_value="turtlebot3_waffle",
            ),
            DeclareLaunchArgument(
                "robot_sdf",
                default_value=os.path.join(
                    simulation_model_share,
                    "urdf",
                    "gz_waffle.sdf.xacro",
                ),
            ),
            DeclareLaunchArgument("x_pose", default_value="-4.15"),
            DeclareLaunchArgument("y_pose", default_value="-3.15"),
            DeclareLaunchArgument("z_pose", default_value="0.01"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("lifecycle_autostart", default_value="true"),
            DeclareLaunchArgument("use_composition", default_value="true"),
            DeclareLaunchArgument("use_respawn", default_value="false"),
            DeclareLaunchArgument(
                "velocity_mux_config",
                default_value=os.path.join(
                    simulation_share,
                    "config",
                    "twist_mux.yaml",
                ),
            ),
            DeclareLaunchArgument("authority_lease_ms", default_value="750"),
            DeclareLaunchArgument(
                "autonomy_velocity_timeout_ms",
                default_value="600",
            ),
            DeclareLaunchArgument(
                "keyboard_velocity_timeout_ms",
                default_value="600",
            ),
            DeclareLaunchArgument(
                "velocity_gate_publish_rate_hz",
                default_value="20.0",
            ),
            DeclareLaunchArgument(
                "velocity_gate_max_linear_x",
                default_value="0.26",
            ),
            DeclareLaunchArgument(
                "velocity_gate_max_angular_z",
                default_value="1.82",
            ),
            DeclareLaunchArgument(
                "persistent_zero_heartbeat_timeout_s",
                default_value="86400.0",
                description=(
                    "How long Collision Monitor keeps publishing final "
                    "zero-velocity heartbeats in a persistent session."
                ),
            ),
            DeclareLaunchArgument(
                "keyboard_authority_source",
                default_value="keyboard_teleop",
            ),
            DeclareLaunchArgument(
                "nav2_progress_radius",
                default_value="0.10",
            ),
            DeclareLaunchArgument(
                "nav2_progress_timeout",
                default_value="30.0",
            ),
            DeclareLaunchArgument(
                "gz_partition",
                default_value=_default_gz_partition(),
            ),
            SetEnvironmentVariable("GZ_PARTITION", gz_partition),
            SetEnvironmentVariable("IGN_PARTITION", gz_partition),
            # Gazebo 与机器人实体归 base 所有，stage 切换时 PID 和 odom 均保持。
            world_xacro,
            # xacro 是异步进程；必须等它成功写完 SDF 后再让 Gazebo 读取，
            # 否则 WSL 负载高时会偶发“空世界/解析失败”。
            RegisterEventHandler(
                OnProcessExit(
                    target_action=world_xacro,
                    on_exit=partial(
                        _handle_world_xacro_exit,
                        generated_world=generated_world,
                        gazebo_server=gazebo_server,
                    ),
                )
            ),
            RegisterEventHandler(
                OnShutdown(
                    on_shutdown=[
                        OpaqueFunction(
                            function=_remove_generated_world,
                            args=[str(generated_world)],
                        )
                    ]
                )
            ),
            _include_launch(
                "ros_gz_sim",
                "gz_sim.launch.py",
                {"gz_args": "-v4 -g"},
                # LaunchConfiguration 展开为小写 true/false，不能直接拼成
                # Python 的 ``not true``；UnlessCondition 会按 ROS 布尔语义解析。
                condition=UnlessCondition(headless),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "robot_description": robot_description,
                    }
                ],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            ),
            _include_launch(
                "nav2_minimal_tb3_sim",
                "spawn_tb3.launch.py",
                {
                    "namespace": "",
                    "use_sim_time": "true",
                    "robot_name": LaunchConfiguration("robot_name"),
                    "robot_sdf": LaunchConfiguration("robot_sdf"),
                    "x_pose": LaunchConfiguration("x_pose"),
                    "y_pose": LaunchConfiguration("y_pose"),
                    "z_pose": LaunchConfiguration("z_pose"),
                    "yaw": LaunchConfiguration("yaw"),
                },
            ),
            _include_launch(
                "nav2_bringup",
                "rviz_launch.py",
                {
                    "namespace": "",
                    "use_namespace": "false",
                    "use_sim_time": "true",
                    "rviz_config": LaunchConfiguration("rviz_config_file"),
                },
                condition=IfCondition(use_rviz),
            ),
            # Nav2 的 planner/controller/BT/Collision Monitor 属于常驻运行时。
            # provider 被显式关闭，后续由可替换 stage 单独接入 SLAM 或 AMCL。
            GroupAction(
                actions=[
                    SetRemap(
                        src="cmd_vel_smoothed",
                        dst="/control/nav2/cmd_vel",
                    ),
                    # Nav2 官方 bringup 会同时加载 docking_server，它默认也直接
                    # 发布 /cmd_vel。用节点作用域 remap 把未使用的泊车速度移出
                    # 最终执行 topic，确保 Collision Monitor 是唯一底盘写者。
                    SetRemap(
                        src="docking_server:/cmd_vel",
                        dst="/control/docking/cmd_vel",
                    ),
                    _include_launch(
                        "nav2_bringup",
                        "bringup_launch.py",
                        {
                            "namespace": "",
                            "use_namespace": "false",
                            # Nav2 官方 bringup 会拼接 ``not <value>``；
                            # 小写 false 会触发 NameError，必须规范为 False。
                            "slam": _as_python_bool("false"),
                            "use_localization": _as_python_bool("false"),
                            "map": "",
                            "use_sim_time": "true",
                            "params_file": configured_nav2_params,
                            "autostart": "true",
                            "use_composition": _as_python_bool(
                                LaunchConfiguration("use_composition")
                            ),
                            "use_respawn": LaunchConfiguration("use_respawn"),
                        },
                    ),
                ]
            ),
            Node(
                package="twist_mux",
                executable="twist_mux",
                name="twist_mux",
                output="screen",
                parameters=[velocity_mux_config, {"use_sim_time": True}],
                remappings=[
                    ("/cmd_vel_out", "/control/selected/cmd_vel"),
                ],
                condition=UnlessCondition(control_authority_enabled),
            ),
            Node(
                package="twist_mux",
                executable="twist_mux",
                name="twist_mux",
                output="screen",
                parameters=[velocity_mux_config, {"use_sim_time": True}],
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
                            authority_lease_ms,
                            value_type=int,
                        ),
                        "autonomy_timeout_ms": ParameterValue(
                            autonomy_velocity_timeout_ms,
                            value_type=int,
                        ),
                        "keyboard_timeout_ms": ParameterValue(
                            keyboard_velocity_timeout_ms,
                            value_type=int,
                        ),
                        "publish_rate_hz": ParameterValue(
                            velocity_gate_publish_rate_hz,
                            value_type=float,
                        ),
                        "max_abs_linear_x": ParameterValue(
                            velocity_gate_max_linear_x,
                            value_type=float,
                        ),
                        "max_abs_angular_z": ParameterValue(
                            velocity_gate_max_angular_z,
                            value_type=float,
                        ),
                        "expected_keyboard_source": LaunchConfiguration(
                            "keyboard_authority_source"
                        ),
                    }
                ],
                condition=IfCondition(control_authority_enabled),
            ),
            # bridge 必须跨阶段常驻，否则切换 executor 时会短暂丢失 Action goal/result。
            LifecycleNode(
                package="embodied_agent_cpp",
                executable="typed_action_bridge",
                name="typed_action_bridge",
                namespace="",
                output="screen",
                # launch_ros autostart 依赖一次性 transition_event。DDS 冷启动
                # 时 service 可能先发现、event 订阅后匹配，导致只 configure
                # 不 activate。持久会话由唯一编排器查询状态并闭环推进转换。
                autostart=False,
            ),
            # Agent 与会话同寿命；mapping/navigation 切换不会重置唤醒和命令队列。
            _include_launch(
                "embodied_online_agent",
                "online_agent.launch.py",
                {
                    **forwarded_agent_launch_arguments(provider_mode),
                    **voice_frontend_config,
                    "authority_gate_enabled": control_authority_enabled,
                    "hardware_enabled": "false",
                    "lifecycle_autostart": lifecycle_autostart,
                },
                condition=online_condition,
            ),
            _include_launch(
                "embodied_offline_agent",
                "offline_agent.launch.py",
                {
                    **forwarded_agent_launch_arguments(provider_mode),
                    **voice_frontend_config,
                    "asr_hotwords_score": LaunchConfiguration(
                        "asr_hotwords_score"
                    ),
                    "authority_gate_enabled": control_authority_enabled,
                    "hardware_enabled": "false",
                    "lifecycle_autostart": lifecycle_autostart,
                },
                condition=offline_condition,
            ),
            # 确定性 stage 验收可关闭 Agent，但 typed action 仍必须经过
            # ActionGuard；否则测试路径会绕开生产安全边界。
            LifecycleNode(
                package="embodied_agent_cpp",
                executable="action_guard",
                name="action_guard",
                namespace="",
                output="screen",
                parameters=[
                    {
                        "authority_gate_enabled": ParameterValue(
                            control_authority_enabled,
                            value_type=bool,
                        )
                    }
                ],
                condition=UnlessCondition(launch_agent),
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="fallback_action_guard_lifecycle_manager",
                output="screen",
                parameters=[
                    {
                        "autostart": ParameterValue(
                            lifecycle_autostart,
                            value_type=bool,
                        ),
                        "node_names": ["action_guard"],
                        "bond_timeout": 0.0,
                    }
                ],
                condition=UnlessCondition(launch_agent),
            ),
        ]
    )
