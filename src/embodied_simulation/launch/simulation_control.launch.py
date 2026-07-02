import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("embodied_simulation"),
        "config",
        "simulation_control.yaml",
    )
    use_typed_actions = LaunchConfiguration("use_typed_actions")
    use_behavior_tree = LaunchConfiguration("use_behavior_tree")
    executor_plugin = LaunchConfiguration("executor_plugin")
    autostart = LaunchConfiguration("autostart")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_typed_actions", default_value="true"),
        DeclareLaunchArgument("use_behavior_tree", default_value="true"),
        DeclareLaunchArgument(
            "executor_plugin",
            default_value="embodied_simulation/GazeboRobotExecutor",
        ),
        DeclareLaunchArgument("autostart", default_value="true"),
        LifecycleNode(
            package="embodied_simulation",
            executable="simulation_control_node",
            name="simulation_control",
            namespace="",
            output="screen",
            parameters=[
                LaunchConfiguration("config"),
                {
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool
                    ),
                    "legacy_command_enabled": ParameterValue(
                        PythonExpression(["'", use_typed_actions, "' != 'true'"]),
                        value_type=bool,
                    ),
                    "use_behavior_tree": ParameterValue(
                        use_behavior_tree, value_type=bool
                    ),
                    "executor_plugin": executor_plugin,
                },
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="simulation_lifecycle_manager",
            output="screen",
            parameters=[{
                "autostart": ParameterValue(autostart, value_type=bool),
                "node_names": ["simulation_control"],
                "bond_timeout": 0.0,
            }],
        ),
        Node(
            package="embodied_agent_cpp",
            executable="typed_action_bridge",
            name="typed_action_bridge",
            output="screen",
            condition=IfCondition(use_typed_actions),
        ),
    ])
