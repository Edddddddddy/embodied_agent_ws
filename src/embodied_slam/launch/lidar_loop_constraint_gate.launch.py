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
    """启动回环约束两阶段门控；commit 默认关闭，只发布审计决策。"""
    autostart = LaunchConfiguration("autostart")
    node = LifecycleNode(
        package="embodied_slam",
        executable="lidar_loop_constraint_gate_node",
        name="lidar_loop_constraint_gate",
        namespace="",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "input_topic": LaunchConfiguration("input_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "commit_enabled": ParameterValue(
                    LaunchConfiguration("commit_enabled"), value_type=bool
                ),
                "require_scan_to_submap": ParameterValue(
                    LaunchConfiguration("require_scan_to_submap"), value_type=bool
                ),
                "minimum_submap_scans": ParameterValue(
                    LaunchConfiguration("minimum_submap_scans"), value_type=int
                ),
                "minimum_correspondences": ParameterValue(
                    LaunchConfiguration("minimum_correspondences"), value_type=int
                ),
                "minimum_descriptor_similarity": ParameterValue(
                    LaunchConfiguration("minimum_descriptor_similarity"),
                    value_type=float,
                ),
                "minimum_inlier_ratio": ParameterValue(
                    LaunchConfiguration("minimum_inlier_ratio"), value_type=float
                ),
                "minimum_overlap_ratio": ParameterValue(
                    LaunchConfiguration("minimum_overlap_ratio"), value_type=float
                ),
                "maximum_rmse_m": ParameterValue(
                    LaunchConfiguration("maximum_rmse_m"), value_type=float
                ),
                "minimum_observability_ratio": ParameterValue(
                    LaunchConfiguration("minimum_observability_ratio"),
                    value_type=float,
                ),
                "minimum_commit_query_separation": ParameterValue(
                    LaunchConfiguration("minimum_commit_query_separation"),
                    value_type=int,
                ),
                "maximum_history": ParameterValue(
                    LaunchConfiguration("maximum_history"), value_type=int
                ),
                "translation_variance": ParameterValue(
                    LaunchConfiguration("translation_variance"), value_type=float
                ),
                "yaw_variance": ParameterValue(
                    LaunchConfiguration("yaw_variance"), value_type=float
                ),
                "enable_multi_hypothesis_sequence": ParameterValue(
                    LaunchConfiguration("enable_multi_hypothesis_sequence"),
                    value_type=bool,
                ),
                "maximum_sequence_hypotheses": ParameterValue(
                    LaunchConfiguration("maximum_sequence_hypotheses"),
                    value_type=int,
                ),
                "minimum_sequence_confirmations": ParameterValue(
                    LaunchConfiguration("minimum_sequence_confirmations"),
                    value_type=int,
                ),
                "maximum_sequence_query_gap_s": ParameterValue(
                    LaunchConfiguration("maximum_sequence_query_gap_s"),
                    value_type=float,
                ),
                "maximum_sequence_pair_age_delta_s": ParameterValue(
                    LaunchConfiguration("maximum_sequence_pair_age_delta_s"),
                    value_type=float,
                ),
                "maximum_sequence_translation_delta_m": ParameterValue(
                    LaunchConfiguration("maximum_sequence_translation_delta_m"),
                    value_type=float,
                ),
                "maximum_sequence_yaw_delta_rad": ParameterValue(
                    LaunchConfiguration("maximum_sequence_yaw_delta_rad"),
                    value_type=float,
                ),
                "minimum_temporal_confirmations": ParameterValue(
                    LaunchConfiguration("minimum_temporal_confirmations"),
                    value_type=int,
                ),
                "maximum_temporal_query_gap_s": ParameterValue(
                    LaunchConfiguration("maximum_temporal_query_gap_s"),
                    value_type=float,
                ),
                "maximum_temporal_pair_age_delta_s": ParameterValue(
                    LaunchConfiguration("maximum_temporal_pair_age_delta_s"),
                    value_type=float,
                ),
                "maximum_temporal_translation_delta_m": ParameterValue(
                    LaunchConfiguration("maximum_temporal_translation_delta_m"),
                    value_type=float,
                ),
                "maximum_temporal_yaw_delta_rad": ParameterValue(
                    LaunchConfiguration("maximum_temporal_yaw_delta_rad"),
                    value_type=float,
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
                LogInfo(msg="Activating LiDAR loop constraint gate"),
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
            DeclareLaunchArgument(
                "input_topic", default_value="/slam/loop_verifications"
            ),
            DeclareLaunchArgument(
                "output_topic", default_value="/slam/loop_constraint_decisions"
            ),
            DeclareLaunchArgument("commit_enabled", default_value="false"),
            DeclareLaunchArgument("require_scan_to_submap", default_value="true"),
            DeclareLaunchArgument("minimum_submap_scans", default_value="2"),
            DeclareLaunchArgument("minimum_correspondences", default_value="30"),
            DeclareLaunchArgument(
                "minimum_descriptor_similarity", default_value="0.60"
            ),
            DeclareLaunchArgument("minimum_inlier_ratio", default_value="0.35"),
            DeclareLaunchArgument("minimum_overlap_ratio", default_value="0.55"),
            DeclareLaunchArgument("maximum_rmse_m", default_value="0.15"),
            DeclareLaunchArgument(
                "minimum_observability_ratio", default_value="0.005"
            ),
            DeclareLaunchArgument(
                "minimum_commit_query_separation", default_value="5"
            ),
            DeclareLaunchArgument("maximum_history", default_value="4096"),
            DeclareLaunchArgument("translation_variance", default_value="0.04"),
            DeclareLaunchArgument("yaw_variance", default_value="0.04"),
            DeclareLaunchArgument(
                "enable_multi_hypothesis_sequence", default_value="true"
            ),
            DeclareLaunchArgument("maximum_sequence_hypotheses", default_value="64"),
            DeclareLaunchArgument("minimum_sequence_confirmations", default_value="3"),
            DeclareLaunchArgument("maximum_sequence_query_gap_s", default_value="2.0"),
            DeclareLaunchArgument(
                "maximum_sequence_pair_age_delta_s", default_value="0.25"
            ),
            DeclareLaunchArgument(
                "maximum_sequence_translation_delta_m", default_value="0.35"
            ),
            DeclareLaunchArgument(
                "maximum_sequence_yaw_delta_rad", default_value="0.20"
            ),
            DeclareLaunchArgument("minimum_temporal_confirmations", default_value="4"),
            DeclareLaunchArgument("maximum_temporal_query_gap_s", default_value="2.0"),
            DeclareLaunchArgument(
                "maximum_temporal_pair_age_delta_s", default_value="1.25"
            ),
            DeclareLaunchArgument(
                "maximum_temporal_translation_delta_m", default_value="0.55"
            ),
            DeclareLaunchArgument(
                "maximum_temporal_yaw_delta_rad", default_value="0.35"
            ),
            node,
            activate,
            configure,
        ]
    )
