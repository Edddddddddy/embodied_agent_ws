"""持久会话的导航阶段：只装载定位 provider 与 Nav2 executor."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _include_launch(package: str, filename: str, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), "launch", filename)
        ),
        launch_arguments=(arguments or {}).items(),
    )


def _as_python_bool(value):
    """把 ROS 小写布尔值转换为 Nav2 PythonExpression 可识别的字面量。"""

    return PythonExpression(
        ["'True' if '", value, "'.lower() == 'true' else 'False'"]
    )


def generate_launch_description():
    simulation_share = get_package_share_directory("embodied_simulation")
    nav2_share = get_package_share_directory("nav2_bringup")
    navigation_share = get_package_share_directory("embodied_navigation")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                description=(
                    "Fresh map YAML produced by the current persistent session."
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
            # provider 默认作为 stage 子进程运行；若加载到 base 的常驻
            # nav2_container，结束 stage launch 时无法保证卸载 AMCL/map_server。
            DeclareLaunchArgument("use_composition", default_value="false"),
            DeclareLaunchArgument("use_respawn", default_value="false"),
            DeclareLaunchArgument("action_timeout_s", default_value="180.0"),
            DeclareLaunchArgument(
                "readiness_stale_timeout_s",
                default_value="30.0",
            ),
            DeclareLaunchArgument(
                "readiness_required_components",
                default_value=(
                    "audio_frontend,agent,action_guard,"
                    "typed_action_bridge,simulation_control"
                ),
                description=(
                    "Set to action_guard,typed_action_bridge,"
                    "simulation_control when base launch_agent=false."
                ),
            ),
            DeclareLaunchArgument(
                "enable_dynamic_obstacle_layer",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "simulation_control_config",
                default_value=os.path.join(
                    simulation_share,
                    "config",
                    "simulation_control.yaml",
                ),
            ),
            # planner/controller/BT 已由 base 持有；map_server 与 AMCL 默认作为
            # stage 独立进程运行，经标准 Nav2 接口接入常驻 common，退出时可完整回收。
            _include_launch(
                "nav2_bringup",
                "localization_launch.py",
                {
                    "namespace": "",
                    "map": LaunchConfiguration("map"),
                    "use_sim_time": "true",
                    "autostart": "true",
                    "params_file": LaunchConfiguration("params_file"),
                    # Nav2 官方 localization launch 会再次用
                    # PythonExpression 拼接参数；这里统一成 True/False，
                    # 防止常见的 ROS 小写 false 被当作未定义 Python 变量。
                    "use_composition": _as_python_bool(
                        LaunchConfiguration("use_composition")
                    ),
                    "use_respawn": _as_python_bool(
                        LaunchConfiguration("use_respawn")
                    ),
                    "container_name": "nav2_container",
                },
            ),
            _include_launch(
                "embodied_simulation",
                "simulation_control.launch.py",
                {
                    "config": LaunchConfiguration(
                        "simulation_control_config"
                    ),
                    "use_sim_time": "true",
                    # typed bridge 常驻在 base；stage 只替换具体 executor。
                    "use_typed_actions": "false",
                    "use_behavior_tree": "true",
                    "executor_plugin": (
                        "embodied_simulation/Nav2RobotExecutor"
                    ),
                    # 与 mapping stage 使用同一 lifecycle owner；新 executor
                    # ACTIVE 后才允许 readiness/AMCL 事务继续。
                    "autostart": "false",
                    "lifecycle_manager_enabled": "false",
                    "action_timeout_s": LaunchConfiguration(
                        "action_timeout_s"
                    ),
                    "cmd_vel_topic": "/control/voice/cmd_vel",
                },
            ),
            # tracker 只在 navigation stage 存活；建图阶段禁止把移动障碍
            # 转成预测代价，避免其轨迹污染待保存的静态地图。
            Node(
                package="embodied_navigation",
                executable="dynamic_obstacle_tracker_node",
                name="dynamic_obstacle_tracker",
                output="screen",
                parameters=[
                    os.path.join(
                        navigation_share,
                        "config",
                        "navigation_overrides.yaml",
                    ),
                    {"use_sim_time": True},
                ],
                condition=IfCondition(
                    LaunchConfiguration(
                        "enable_dynamic_obstacle_layer"
                    )
                ),
            ),
            # 使用独立 profile 与全新的健康注册表，阻断 mapping 阶段迟到的
            # transient-local ready 消息放行 AMCL/Nav2。
            Node(
                package="embodied_agent_middleware",
                executable="system_readiness_node",
                name="system_readiness",
                output="screen",
                parameters=[
                    {
                        "profile": "persistent_navigation_stage",
                        "required_components_csv": LaunchConfiguration(
                            "readiness_required_components"
                        ),
                        "stale_timeout_s": ParameterValue(
                            LaunchConfiguration(
                                "readiness_stale_timeout_s"
                            ),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
