"""SLAM/Nav2 probe 的时间适配与 sampled plan 关联契约。"""

import threading
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from embodied_agent_interfaces.msg import (
    SlamNavigationGoalEvidence,
    SlamSessionState,
)
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path as NavPath

from tools.acceptance.probes.slam_nav.localization_sampler import (
    GazeboTruthConfig,
    LocalizationSampler,
)
from tools.acceptance.probes.slam_nav.motion_evidence import MotionEvidenceTracker
from tools.acceptance.probes.slam_nav.sampled_goal_tracker import (
    SampledGoalPlanTracker,
    _PENDING_SAMPLED_PLAN_LIMIT,
    _sampled_goal_message_to_dict,
)
from tools.acceptance.probes.slam_nav.session_observer import (
    SessionObserver,
    _gazebo_timestamp_s,
    _ros_timestamp_s,
)
from tools.acceptance.unknown_world_evidence import MapWorldTransform


def test_ros_time_uses_nanosec_field():
    stamp = Time(sec=3, nanosec=250_000_000)

    assert _ros_timestamp_s(stamp) == 3.25


def test_gazebo_protobuf_time_uses_nsec_field():
    stamp = SimpleNamespace(sec=4, nsec=500_000_000)

    assert _gazebo_timestamp_s(stamp) == 4.5


def _observer_without_ros_node() -> SessionObserver:
    observer = SessionObserver.__new__(SessionObserver)
    observer._sampled_goal_tracker = SampledGoalPlanTracker()
    return observer


def _goal(
    sequence: int,
    x: float,
    y: float,
    status: int,
    *,
    started_at_s: int = 10,
    finished_at_s: int = 0,
) -> SlamNavigationGoalEvidence:
    goal = SlamNavigationGoalEvidence()
    goal.sequence = sequence
    goal.goal.header.frame_id = "map"
    goal.goal.pose.position.x = x
    goal.goal.pose.position.y = y
    goal.status = status
    goal.started_at = Time(sec=started_at_s)
    goal.finished_at = Time(sec=finished_at_s)
    return goal


def _state(
    mission_sequence: int,
    *goals: SlamNavigationGoalEvidence,
) -> SlamSessionState:
    state = SlamSessionState()
    state.mission_sequence = mission_sequence
    state.sampled_navigation_goals = list(goals)
    return state


def _path(
    x: float,
    y: float,
    *,
    frame_id: str = "map",
    stamp_s: int = 11,
    pose_frame_id: str = "map",
) -> NavPath:
    path = NavPath()
    path.header.frame_id = frame_id
    path.header.stamp = Time(sec=stamp_s)
    pose = PoseStamped()
    pose.header.frame_id = pose_frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    path.poses.append(pose)
    return path


def test_sampled_goal_adapter_preserves_lifecycle_timestamps():
    snapshot = _sampled_goal_message_to_dict(
        _goal(
            3,
            1.0,
            2.0,
            SlamNavigationGoalEvidence.STATUS_SUCCEEDED,
            started_at_s=12,
            finished_at_s=18,
        )
    )

    assert snapshot["started_at"] == 12.0
    assert snapshot["finished_at"] == 18.0


def test_plan_requires_map_frame_and_non_empty_path():
    observer = _observer_without_ros_node()
    observer._update_sampled_goal_evidence(
        _state(
            7,
            _goal(
                1,
                1.0,
                2.0,
                SlamNavigationGoalEvidence.STATUS_EXECUTING,
            ),
        )
    )

    observer._on_navigation_plan(NavPath())
    observer._on_navigation_plan(_path(1.0, 2.0, frame_id="odom"))
    observer._on_navigation_plan(_path(1.0, 2.0, pose_frame_id="odom"))

    assert observer.navigation_plans == []
    assert observer.sampled_goal_plans == {}
    assert len(observer._sampled_goal_tracker.pending_plans) == 0


def test_plan_is_bound_only_to_active_typed_goal_with_matching_endpoint():
    observer = _observer_without_ros_node()
    observer._update_sampled_goal_evidence(
        _state(
            7,
            _goal(
                2,
                1.0,
                2.0,
                SlamNavigationGoalEvidence.STATUS_EXECUTING,
            ),
        )
    )

    unrelated = _path(-3.0, 4.0)
    expected = _path(1.1, 2.1)
    observer._on_navigation_plan(unrelated)
    observer._on_navigation_plan(expected)

    assert observer.sampled_goal_plans == {2: [expected]}
    assert list(observer._sampled_goal_tracker.pending_plans) != []


def test_plan_before_typed_state_is_replayed_from_bounded_buffer():
    observer = _observer_without_ros_node()
    early_plan = _path(1.0, 2.0)

    observer._on_navigation_plan(early_plan)
    observer._update_sampled_goal_evidence(
        _state(
            7,
            _goal(
                1,
                1.0,
                2.0,
                SlamNavigationGoalEvidence.STATUS_REQUESTED,
            ),
        )
    )

    assert observer.sampled_goal_plans == {1: [early_plan]}
    assert len(observer._sampled_goal_tracker.pending_plans) == 0


def test_late_previous_goal_plan_cannot_wake_or_bind_next_goal():
    observer = _observer_without_ros_node()
    first = _goal(
        1,
        1.0,
        2.0,
        SlamNavigationGoalEvidence.STATUS_EXECUTING,
    )
    observer._update_sampled_goal_evidence(_state(7, first))
    first_plan = _path(1.0, 2.0)
    observer._on_navigation_plan(first_plan)

    finished = _goal(
        1,
        1.0,
        2.0,
        SlamNavigationGoalEvidence.STATUS_SUCCEEDED,
        finished_at_s=14,
    )
    second = _goal(
        2,
        3.0,
        4.0,
        SlamNavigationGoalEvidence.STATUS_EXECUTING,
        started_at_s=15,
    )
    observer._update_sampled_goal_evidence(_state(7, finished, second))
    observer._on_navigation_plan(_path(1.0, 2.0, stamp_s=16))
    second_plan = _path(3.0, 4.0, stamp_s=16)
    observer._on_navigation_plan(second_plan)

    assert observer.sampled_goal_plans == {1: [first_plan], 2: [second_plan]}


def test_mission_change_drops_old_pending_plan_even_at_same_endpoint():
    observer = _observer_without_ros_node()
    terminal = _goal(
        1,
        1.0,
        2.0,
        SlamNavigationGoalEvidence.STATUS_SUCCEEDED,
        finished_at_s=15,
    )
    observer._update_sampled_goal_evidence(_state(7, terminal))
    observer._on_navigation_plan(_path(1.0, 2.0, stamp_s=16))

    next_mission = _goal(
        1,
        1.0,
        2.0,
        SlamNavigationGoalEvidence.STATUS_EXECUTING,
        started_at_s=16,
    )
    observer._update_sampled_goal_evidence(_state(8, next_mission))

    assert observer.sampled_goal_plans == {}
    assert len(observer._sampled_goal_tracker.pending_plans) == 0


def test_plan_older_than_typed_goal_lifecycle_is_not_bound():
    observer = _observer_without_ros_node()
    observer._update_sampled_goal_evidence(
        _state(
            7,
            _goal(
                1,
                1.0,
                2.0,
                SlamNavigationGoalEvidence.STATUS_EXECUTING,
                started_at_s=20,
            ),
        )
    )

    observer._on_navigation_plan(_path(1.0, 2.0, stamp_s=19))
    current = _path(1.0, 2.0, stamp_s=21)
    observer._on_navigation_plan(current)

    assert observer.sampled_goal_plans == {1: [current]}


def test_sim_time_plan_can_bind_wall_time_typed_goal():
    observer = _observer_without_ros_node()
    observer._update_sampled_goal_evidence(
        _state(
            7,
            _goal(
                1,
                1.0,
                2.0,
                SlamNavigationGoalEvidence.STATUS_EXECUTING,
                # Session orchestrator 使用系统时钟，而 Nav2 在 Gazebo 中使用 /clock。
                started_at_s=1_784_505_062,
            ),
        )
    )

    simulated_plan = _path(1.0, 2.0, stamp_s=700)
    observer._on_navigation_plan(simulated_plan)

    # 两个时间值不在同一时钟域，不能据绝对大小把真实执行路径误判为旧消息。
    assert observer.sampled_goal_plans == {1: [simulated_plan]}


def test_cross_clock_plan_before_typed_state_is_replayed():
    observer = _observer_without_ros_node()
    simulated_plan = _path(1.0, 2.0, stamp_s=700)

    observer._on_navigation_plan(simulated_plan)
    observer._update_sampled_goal_evidence(
        _state(
            7,
            _goal(
                1,
                1.0,
                2.0,
                SlamNavigationGoalEvidence.STATUS_REQUESTED,
                started_at_s=1_784_505_062,
            ),
        )
    )

    # 跨 topic 的正常乱序仍受 4s TTL 和 mission/endpoint 约束，不能因时钟域
    # 不同而丢掉先到达的一条真实 Nav2 规划。
    assert observer.sampled_goal_plans == {1: [simulated_plan]}
    assert len(observer._sampled_goal_tracker.pending_plans) == 0


def test_pending_plan_buffer_is_bounded():
    observer = _observer_without_ros_node()

    for index in range(_PENDING_SAMPLED_PLAN_LIMIT + 5):
        observer._on_navigation_plan(_path(float(index), 0.0))

    assert (
        len(observer._sampled_goal_tracker.pending_plans)
        == _PENDING_SAMPLED_PLAN_LIMIT
    )


def test_sampled_state_and_plan_callbacks_are_thread_safe():
    observer = _observer_without_ros_node()
    active_state = _state(
        7,
        _goal(
            1,
            1.0,
            2.0,
            SlamNavigationGoalEvidence.STATUS_EXECUTING,
        ),
    )
    observer._update_sampled_goal_evidence(active_state)
    barrier = threading.Barrier(2)

    def publish_states() -> None:
        barrier.wait()
        for _ in range(40):
            observer._update_sampled_goal_evidence(active_state)

    def publish_plans() -> None:
        barrier.wait()
        for _ in range(40):
            observer._on_navigation_plan(_path(1.0, 2.0))

    threads = [
        threading.Thread(target=publish_states),
        threading.Thread(target=publish_plans),
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=2.0)

    assert all(not thread.is_alive() for thread in threads)
    assert len(observer.sampled_goal_plans[1]) == 40


def test_tracker_snapshot_is_consistent_and_does_not_expose_mutable_lists():
    tracker = SampledGoalPlanTracker()
    tracker.update_state(
        _state(
            7,
            _goal(
                1,
                1.0,
                2.0,
                SlamNavigationGoalEvidence.STATUS_EXECUTING,
            ),
        )
    )
    plan = _path(1.0, 2.0)
    tracker.observe_plan(plan)

    snapshot = tracker.snapshot()

    assert snapshot.mission_sequence == 7
    assert snapshot.active_sequence == 1
    assert snapshot.plans == {1: (plan,)}
    assert snapshot.navigation_plans == (plan,)


def test_terminal_stop_requires_a_new_velocity_sample_after_terminal_state():
    observer = SessionObserver.__new__(SessionObserver)
    observer._motion_evidence_tracker = MotionEvidenceTracker()
    observer._motion_evidence_tracker.mark_boundary(boundary_at_s=100.0)

    assert observer.has_fresh_terminal_stop() is False

    moving = Twist()
    moving.linear.x = 0.2
    observer._on_cmd_vel(moving)
    assert observer.nonzero_cmd_vel_sample_count == 1

    # 测试使用人工 monotonic 时间边界；真正回调会取 time.monotonic()。
    observer._motion_evidence_tracker.mark_boundary()
    stopped = Twist()
    observer._on_cmd_vel(stopped)

    assert observer.has_fresh_terminal_stop() is True
    assert observer.has_fresh_final_motion_stop() is False

    observer.mark_final_stop_boundary()
    observer._on_cmd_vel(moving)
    observer._on_cmd_vel(stopped)
    assert observer.has_fresh_final_motion_stop() is True


def _amcl(timestamp_s: int, x: float, y: float) -> PoseWithCovarianceStamped:
    message = PoseWithCovarianceStamped()
    message.header.stamp = Time(sec=timestamp_s)
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    return message


def _gazebo_pose_vector(timestamp_s: int, x: float, y: float):
    pose = SimpleNamespace(
        name="turtlebot3_waffle",
        id=42,
        position=SimpleNamespace(x=x, y=y),
    )
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=timestamp_s, nsec=0)),
        pose=[pose],
    )


def _localization_sampler() -> LocalizationSampler:
    sampler = LocalizationSampler(
        GazeboTruthConfig(
            world_name="test_world",
            robot_entity_name="turtlebot3_waffle",
            transform=MapWorldTransform(0.0, 0.0, 0.0),
        )
    )
    sampler.transition_phase(SlamSessionState.NAVIGATING)
    return sampler


def test_amcl_redelivery_is_deduplicated_but_conflicting_pose_is_an_error():
    sampler = _localization_sampler()

    sampler.observe_amcl(_amcl(10, 1.0, 2.0))
    sampler.observe_amcl(_amcl(10, 1.0, 2.0))
    assert len(sampler.evidence()[0]) == 1
    assert sampler.evidence()[2] == ""

    sampler.observe_amcl(_amcl(10, 1.1, 2.0))
    assert len(sampler.evidence()[0]) == 1
    assert "AMCL has conflicting poses" in sampler.evidence()[2]


def test_gazebo_redelivery_is_deduplicated_but_conflicting_pose_is_an_error():
    sampler = _localization_sampler()

    sampler.observe_gazebo(_gazebo_pose_vector(10, 1.0, 2.0))
    sampler.observe_gazebo(_gazebo_pose_vector(10, 1.0, 2.0))
    assert len(sampler.evidence()[1]) == 1
    assert sampler.evidence()[2] == ""

    sampler.observe_gazebo(_gazebo_pose_vector(10, 1.1, 2.0))
    assert len(sampler.evidence()[1]) == 1
    assert "Gazebo truth has conflicting poses" in sampler.evidence()[2]


def test_mapping_to_navigation_transition_starts_a_fresh_localization_stage():
    sampler = _localization_sampler()
    sampler.observe_amcl(_amcl(10, 1.0, 2.0))
    sampler.observe_gazebo(_gazebo_pose_vector(10, 1.0, 2.0))

    sampler.transition_phase(SlamSessionState.AUTOMATIC_MAPPING)
    sampler.observe_amcl(_amcl(11, 9.0, 9.0))
    sampler.transition_phase(SlamSessionState.STARTING_NAVIGATION)

    amcl, gazebo, error = sampler.evidence()
    assert amcl == ()
    assert gazebo == ()
    assert error == ""
