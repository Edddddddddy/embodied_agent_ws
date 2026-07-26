import pytest

from embodied_slam_tools.control_authority_lease import (
    AUTONOMY,
    ESTOP,
    HOLD,
    KEYBOARD,
    AuthoritySnapshot,
    AuthorityUpdateDecision,
    ControlAuthorityLease,
)


def _state(
    authority,
    sequence,
    *,
    epoch=1,
    source=None,
    reason="test",
    pending_revocation=0,
    quiescence_acknowledged=None,
):
    if source is None:
        source = {
            HOLD: "",
            AUTONOMY: "autonomy",
            KEYBOARD: "keyboard_teleop",
            ESTOP: "safety_panel",
        }[authority]
    if authority == HOLD and sequence == 0:
        reason = "initialized"
    if quiescence_acknowledged is None:
        quiescence_acknowledged = authority != AUTONOMY
    return AuthoritySnapshot(
        authority=authority,
        estop_latched=authority == ESTOP,
        manager_epoch=epoch,
        transition_sequence=sequence,
        active_source=source,
        reason=reason,
        pending_autonomy_revocation_sequence=pending_revocation,
        autonomy_quiescence_acknowledged=quiescence_acknowledged,
    )


def test_authority_lease_rejects_old_and_conflicting_states():
    tracker = ControlAuthorityLease(lease_s=0.5)
    assert tracker.update(_state(AUTONOMY, 2), 1.0).accepted

    old = tracker.update(_state(HOLD, 1), 1.1)
    conflict = tracker.update(_state(KEYBOARD, 2), 1.2)

    assert not old.accepted
    assert old.decision == AuthorityUpdateDecision.REJECTED_OLD_SEQUENCE
    assert not conflict.accepted
    assert (
        conflict.decision
        == AuthorityUpdateDecision.REJECTED_CONFLICTING_HEARTBEAT
    )
    assert tracker.authority == AUTONOMY


def test_authority_lease_gap_is_sticky_until_a_new_manager_epoch():
    tracker = ControlAuthorityLease(lease_s=0.5)
    autonomy = _state(AUTONOMY, 1)
    assert tracker.update(autonomy, 1.0).accepted
    assert tracker.generation == 1
    assert not tracker.fresh(1.501)

    late_heartbeat = tracker.update(autonomy, 1.501)
    late_transition = tracker.update(_state(HOLD, 2), 1.6)

    assert not late_heartbeat.accepted
    assert late_heartbeat.lease_discontinuity
    assert (
        late_heartbeat.decision
        == AuthorityUpdateDecision.REJECTED_LEASE_DISCONTINUITY
    )
    assert not late_transition.accepted
    assert (
        late_transition.decision
        == AuthorityUpdateDecision.REJECTED_LEASE_DISCONTINUITY
    )
    assert tracker.generation == 1
    assert not tracker.fresh_autonomy(1.502)

    restarted = tracker.update(_state(HOLD, 0, epoch=2), 1.7)
    assert restarted.accepted
    assert restarted.generation_changed
    assert tracker.generation == 2
    assert tracker.authority == HOLD


def test_authority_manager_restart_must_bootstrap_hold_and_cannot_roll_back():
    tracker = ControlAuthorityLease(lease_s=0.5)
    assert tracker.update(_state(AUTONOMY, 4, epoch=10), 1.0).accepted
    assert not tracker.update(_state(AUTONOMY, 1, epoch=20), 1.1).accepted

    restarted = tracker.update(_state(HOLD, 0, epoch=20), 1.2)
    assert restarted.accepted
    assert restarted.lease_discontinuity
    assert tracker.authority == HOLD

    rollback = tracker.update(_state(HOLD, 0, epoch=10), 1.3)
    assert not rollback.accepted
    assert rollback.decision == AuthorityUpdateDecision.REJECTED_RETIRED_EPOCH


def test_authority_invalid_estop_does_not_refresh_lease():
    tracker = ControlAuthorityLease(lease_s=0.5)
    assert tracker.update(_state(AUTONOMY, 1), 1.0).accepted
    malformed = AuthoritySnapshot(
        authority=ESTOP,
        estop_latched=False,
        manager_epoch=1,
        transition_sequence=2,
        active_source="safety_panel",
        reason="bad",
    )

    assert not tracker.update(malformed, 1.4).accepted
    assert not tracker.fresh(1.501)


@pytest.mark.parametrize(
    ("pending_revocation", "quiescence_acknowledged"),
    [
        (1, False),
        (0, True),
    ],
)
def test_autonomy_payload_matches_cpp_contract_and_cannot_refresh_lease(
    pending_revocation,
    quiescence_acknowledged,
):
    tracker = ControlAuthorityLease(lease_s=0.5)
    assert tracker.update(_state(AUTONOMY, 1), 1.0).accepted
    generation = tracker.generation

    malformed = _state(
        AUTONOMY,
        2,
        pending_revocation=pending_revocation,
        quiescence_acknowledged=quiescence_acknowledged,
    )
    update = tracker.update(malformed, 1.4)

    assert not update.accepted
    assert update.decision == AuthorityUpdateDecision.REJECTED_INVALID_PAYLOAD
    assert tracker.generation == generation
    assert not tracker.fresh(1.501)


def test_authority_lease_rejects_non_positive_configuration():
    with pytest.raises(ValueError):
        ControlAuthorityLease(lease_s=0.0)
