import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


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


def generate_launch_description():
    simulation_share = get_package_share_directory("embodied_simulation")
    nav2_share = get_package_share_directory("nav2_bringup")
    nav2_params = os.path.join(nav2_share, "params", "nav2_params.yaml")
    nav2_map = os.path.join(nav2_share, "maps", "tb3_sandbox.yaml")
    control_config = os.path.join(
        simulation_share, "config", "simulation_control.yaml"
    )

    launch_agent = LaunchConfiguration("launch_agent")
    agent_type = LaunchConfiguration("agent_type")
    provider_mode = LaunchConfiguration("provider_mode")
    microphone = LaunchConfiguration("microphone_enabled")
    continuous_control = LaunchConfiguration("continuous_control_enabled")
    wake_word = LaunchConfiguration("wake_word_enabled")
    lifecycle_autostart = LaunchConfiguration("lifecycle_autostart")
    nav_action_timeout_s = LaunchConfiguration("nav_action_timeout_s")
    slam = LaunchConfiguration("slam")
    use_rviz = LaunchConfiguration("use_rviz")
    headless = LaunchConfiguration("headless")
    use_composition = LaunchConfiguration("use_composition")

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

    return LaunchDescription([
        DeclareLaunchArgument("launch_agent", default_value="true"),
        DeclareLaunchArgument("agent_type", default_value="online"),
        DeclareLaunchArgument("provider_mode", default_value="mock"),
        DeclareLaunchArgument("microphone_enabled", default_value="false"),
        DeclareLaunchArgument("continuous_control_enabled", default_value="false"),
        DeclareLaunchArgument("wake_word_enabled", default_value="true"),
        DeclareLaunchArgument("lifecycle_autostart", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("slam", default_value="false"),
        DeclareLaunchArgument("map", default_value=nav2_map),
        DeclareLaunchArgument("params_file", default_value=nav2_params),
        DeclareLaunchArgument("use_composition", default_value="true"),
        DeclareLaunchArgument("nav_action_timeout_s", default_value="180.0"),
        DeclareLaunchArgument("x_pose", default_value="-2.0"),
        DeclareLaunchArgument("y_pose", default_value="-0.5"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        # 复用 Nav2 官方 TurtleBot3 仿真 bringup：地图、AMCL/SLAM、planner/controller
        # 都由 nav2_bringup 管理，本项目只接入“语音 -> Nav2 action”的上层链路。
        include_launch(
            "nav2_bringup",
            "tb3_simulation_launch.py",
            {
                # Nav2 官方 launch 内部使用 PythonExpression(['not ', use_composition])
                # 这类表达式，必须收到 Python 可识别的 True/False；本项目对外仍保留
                # ROS 常见的小写 true/false 参数，避免用户命令行习惯被打破。
                "slam": as_python_bool(slam),
                "map": LaunchConfiguration("map"),
                "params_file": LaunchConfiguration("params_file"),
                "use_rviz": as_python_bool(use_rviz),
                "headless": as_python_bool(headless),
                "autostart": "true",
                "use_sim_time": "true",
                "use_composition": as_python_bool(use_composition),
                "x_pose": LaunchConfiguration("x_pose"),
                "y_pose": LaunchConfiguration("y_pose"),
                "yaw": LaunchConfiguration("yaw"),
            },
        ),
        include_launch(
            "embodied_simulation",
            "simulation_control.launch.py",
            {
                "config": control_config,
                "use_sim_time": "true",
                "use_typed_actions": "true",
                "use_behavior_tree": "true",
                "executor_plugin": "embodied_simulation/Nav2RobotExecutor",
                "autostart": lifecycle_autostart,
                "action_timeout_s": nav_action_timeout_s,
            },
        ),
        include_launch(
            "embodied_online_agent",
            "online_agent.launch.py",
            {
                "mode": provider_mode,
                "microphone_enabled": microphone,
                "wake_word_enabled": wake_word,
                "continuous_control_enabled": continuous_control,
                "hardware_enabled": "false",
                "lifecycle_autostart": lifecycle_autostart,
            },
            online_condition,
        ),
        include_launch(
            "embodied_offline_agent",
            "offline_agent.launch.py",
            {
                "mode": provider_mode,
                "microphone_enabled": microphone,
                "wake_word_enabled": wake_word,
                "continuous_control_enabled": continuous_control,
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
