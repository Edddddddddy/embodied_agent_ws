from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    """启动只发布候选证据、绝不直接写位姿图的在线 LiDAR 前端。"""
    autostart = LaunchConfiguration("autostart")
    node = LifecycleNode(
        package="embodied_slam",
        executable="lidar_loop_candidate_node",
        name="lidar_loop_candidate",
        namespace="",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "input_topic": LaunchConfiguration("input_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "minimum_temporal_separation_s": ParameterValue(
                    LaunchConfiguration("minimum_temporal_separation_s"),
                    value_type=float,
                ),
                "minimum_similarity": ParameterValue(
                    LaunchConfiguration("minimum_similarity"), value_type=float
                ),
                "sample_interval_s": ParameterValue(
                    LaunchConfiguration("sample_interval_s"), value_type=float
                ),
                "top_k": ParameterValue(
                    LaunchConfiguration("top_k"), value_type=int
                ),
            }
        ],
    )
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(autostart),
    )
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="Activating shadow LiDAR loop candidate frontend"),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(autostart),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("input_topic", default_value="/scan"),
            DeclareLaunchArgument(
                "output_topic", default_value="/slam/loop_candidates"
            ),
            DeclareLaunchArgument(
                "minimum_temporal_separation_s", default_value="60.0"
            ),
            DeclareLaunchArgument("minimum_similarity", default_value="0.0"),
            DeclareLaunchArgument("sample_interval_s", default_value="0.5"),
            DeclareLaunchArgument("top_k", default_value="10"),
            node,
            activate,
            configure,
        ]
    )
