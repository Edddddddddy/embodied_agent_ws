"""Autonomy quiescence barrier tests without ROS runtime."""

import pytest

from embodied_slam_tools.autonomy_quiescence import (
    AutonomyQuiescenceBarrier,
    QuiescenceError,
    QuiescenceIdentity,
    QuiescenceState,
    QuiescenceTimeout,
)


def _begin_idle_barrier() -> tuple[AutonomyQuiescenceBarrier, QuiescenceIdentity]:
    barrier = AutonomyQuiescenceBarrier()
    identity = QuiescenceIdentity(manager_epoch=7, revocation_sequence=11)
    barrier.observe_authority(identity)
    barrier.begin(identity)
    return barrier, identity


def _ready_barrier() -> tuple[AutonomyQuiescenceBarrier, QuiescenceIdentity]:
    barrier, identity = _begin_idle_barrier()
    barrier.mark_priority_stop_requested(
        "quiescence-stop-7-11", cmd_vel_generation=4
    )
    barrier.observe_priority_stop_result(
        "quiescence-stop-7-11", success=True, detail="stopped"
    )
    barrier.observe_cmd_vel(generation=5, linear_x=0.0, angular_z=0.0)
    assert barrier.wait_ready(timeout_s=0.0).ready
    return barrier, identity


def test_ack_in_flight_accepts_fast_manager_resume_idempotently():
    barrier, identity = _ready_barrier()

    barrier.begin_acknowledgement(identity)
    assert barrier.snapshot().state is QuiescenceState.ACK_IN_FLIGHT

    # manager 只有在接受同代 ACK 后才能发布 AUTONOMY；topic callback 可能先于
    # service future callback 被 executor 调度，不能因此把合法快速恢复误判为失败。
    barrier.observe_authority(
        QuiescenceIdentity(identity.manager_epoch, 0),
        autonomy_active=True,
    )
    assert barrier.snapshot().state is QuiescenceState.ACKNOWLEDGED

    assert not barrier.complete_acknowledgement(identity)
    assert barrier.snapshot().state is QuiescenceState.ACKNOWLEDGED


def test_ack_in_flight_fails_closed_when_zero_evidence_is_invalidated():
    barrier, identity = _ready_barrier()
    barrier.begin_acknowledgement(identity)

    barrier.observe_cmd_vel(generation=6, linear_x=0.08, angular_z=0.0)

    assert barrier.snapshot().state is QuiescenceState.FAILED
    with pytest.raises(QuiescenceError, match="evidence invalidated"):
        barrier.complete_acknowledgement(identity)


def test_next_revocation_can_supersede_only_an_ack_in_flight_identity():
    barrier, first = _ready_barrier()
    barrier.begin_acknowledgement(first)
    second = QuiescenceIdentity(first.manager_epoch, first.revocation_sequence + 3)

    # executor 可先调度第二次 HOLD，而遗漏中间 ACK/AUTONOMY topic；严格递增的
    # manager 状态证明旧 ACK 已被接受，随后第二代必须立即建立独立屏障。
    barrier.observe_authority(second)
    assert barrier.snapshot().state is QuiescenceState.ACKNOWLEDGED
    barrier.begin(second)

    assert barrier.snapshot().identity == second
    assert barrier.snapshot().state is QuiescenceState.WAITING
    assert not barrier.complete_acknowledgement(first)


def test_idle_takeover_requires_exact_priority_stop_and_post_request_zero():
    barrier, identity = _begin_idle_barrier()

    barrier.mark_priority_stop_requested(
        "quiescence-stop-7-11", cmd_vel_generation=4
    )
    barrier.observe_cmd_vel(generation=4, linear_x=0.0, angular_z=0.0)
    barrier.observe_priority_stop_result(
        "quiescence-stop-7-11", success=True, detail="stopped"
    )

    assert barrier.snapshot().ready is False
    assert "fresh_zero_cmd_vel" in barrier.snapshot().missing

    barrier.observe_cmd_vel(generation=5, linear_x=0.0, angular_z=0.0)

    ready = barrier.wait_ready(timeout_s=0.0)
    assert ready.identity == identity
    assert ready.ready
    barrier.begin_acknowledgement(identity)
    barrier.complete_acknowledgement(identity)
    assert barrier.snapshot().state is QuiescenceState.ACKNOWLEDGED


def test_active_explore_and_nav2_must_both_reach_typed_terminal():
    barrier = AutonomyQuiescenceBarrier()
    barrier.observe_frontier(
        status="exploration_running",
        active_goal_count=1,
        accepted_goal_count=2,
        terminal_goal_count=1,
    )
    barrier.nav2_goal_started("navigate:mission-2:goal-1")
    identity = QuiescenceIdentity(9, 3)
    barrier.observe_authority(identity)
    barrier.begin(identity)
    barrier.mark_priority_stop_requested("stop-9-3", cmd_vel_generation=8)
    barrier.observe_priority_stop_result(
        "stop-9-3", success=True, detail="stopped"
    )
    barrier.observe_cmd_vel(generation=9, linear_x=0.0, angular_z=0.0)

    assert set(barrier.snapshot().missing) == {
        "explore_ledger_terminal",
        "nav2_action_terminal",
    }

    barrier.observe_frontier(
        status="exploration_paused",
        active_goal_count=0,
        accepted_goal_count=2,
        terminal_goal_count=2,
    )
    assert barrier.snapshot().missing == ("nav2_action_terminal",)

    barrier.nav2_goal_terminal("navigate:mission-2:goal-1")
    assert barrier.wait_ready(timeout_s=0.0).ready


def test_new_nav2_goal_after_revocation_fails_closed():
    barrier, _identity = _begin_idle_barrier()

    barrier.nav2_goal_started("ghost-goal")

    snapshot = barrier.snapshot()
    assert snapshot.state is QuiescenceState.FAILED
    assert "started after autonomy revocation" in snapshot.failure_reason


@pytest.mark.parametrize(
    "observed",
    [
        QuiescenceIdentity(manager_epoch=8, revocation_sequence=11),
        QuiescenceIdentity(manager_epoch=7, revocation_sequence=12),
    ],
)
def test_authority_epoch_or_revocation_change_fails_closed(observed):
    barrier, _identity = _begin_idle_barrier()

    barrier.observe_authority(observed)

    snapshot = barrier.snapshot()
    assert snapshot.state is QuiescenceState.FAILED
    assert "authority identity changed" in snapshot.failure_reason


def test_failed_priority_stop_never_allows_acknowledgement():
    barrier, identity = _begin_idle_barrier()
    barrier.mark_priority_stop_requested("stop-7-11", cmd_vel_generation=1)

    barrier.observe_priority_stop_result(
        "stop-7-11", success=False, detail="executor rejected stop"
    )

    with pytest.raises(QuiescenceError, match="priority STOP failed"):
        barrier.wait_ready(timeout_s=0.0)
    with pytest.raises(QuiescenceError):
        barrier.mark_acknowledged(identity)


def test_nonzero_velocity_after_zero_requires_another_fresh_zero():
    barrier, _identity = _begin_idle_barrier()
    barrier.mark_priority_stop_requested("stop-7-11", cmd_vel_generation=1)
    barrier.observe_priority_stop_result("stop-7-11", success=True)
    barrier.observe_cmd_vel(generation=2, linear_x=0.0, angular_z=0.0)
    assert barrier.snapshot().ready

    barrier.observe_cmd_vel(generation=3, linear_x=0.1, angular_z=0.0)
    assert "fresh_zero_cmd_vel" in barrier.snapshot().missing

    barrier.observe_cmd_vel(generation=4, linear_x=0.0, angular_z=0.0)
    assert barrier.snapshot().ready


def test_timeout_is_terminal_and_never_becomes_ready_later():
    barrier, _identity = _begin_idle_barrier()

    with pytest.raises(QuiescenceTimeout, match="priority_stop_result"):
        barrier.wait_ready(timeout_s=0.0)

    with pytest.raises(QuiescenceError):
        barrier.mark_priority_stop_requested(
            "stop-7-11", cmd_vel_generation=1
        )
    barrier.observe_priority_stop_result("stop-7-11", success=True)
    barrier.observe_cmd_vel(generation=2, linear_x=0.0, angular_z=0.0)
    assert barrier.snapshot().state is QuiescenceState.FAILED
    assert not barrier.snapshot().ready


def test_stale_terminal_from_before_revocation_fails_closed():
    barrier = AutonomyQuiescenceBarrier()
    barrier.nav2_goal_started("goal-1")
    barrier.nav2_goal_terminal("goal-1")
    barrier.nav2_goal_started("goal-2")
    identity = QuiescenceIdentity(3, 4)
    barrier.observe_authority(identity)
    barrier.begin(identity)

    with pytest.raises(QuiescenceError, match="duplicate Nav2 terminal"):
        barrier.nav2_goal_terminal("goal-1")

    assert barrier.snapshot().state is QuiescenceState.FAILED
    assert "goal-1" in barrier.snapshot().failure_reason


def test_unknown_nav2_terminal_token_fails_closed():
    barrier, _identity = _begin_idle_barrier()

    with pytest.raises(QuiescenceError, match="unknown Nav2 terminal"):
        barrier.nav2_goal_terminal("never-started")

    assert barrier.snapshot().state is QuiescenceState.FAILED
