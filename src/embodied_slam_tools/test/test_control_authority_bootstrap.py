from types import SimpleNamespace

from embodied_agent_interfaces.msg import ControlAuthorityState
from embodied_slam_tools.control_authority_bootstrap import (
    validate_cold_bootstrap_state,
    validate_resumed_state,
)
import pytest


def _state(
    authority,
    sequence,
    *,
    epoch=41,
    acknowledged=False,
    pending=0,
    source='',
    estop=False,
):
    return SimpleNamespace(
        authority=authority,
        estop_latched=estop,
        manager_epoch=epoch,
        transition_sequence=sequence,
        pending_autonomy_revocation_sequence=pending,
        autonomy_quiescence_acknowledged=acknowledged,
        active_source=source,
    )


def test_cold_bootstrap_requires_exact_initial_hold_permission():
    state = _state(
        ControlAuthorityState.HOLD,
        0,
        acknowledged=True,
    )

    assert validate_cold_bootstrap_state(state) == (41, 0)


@pytest.mark.parametrize(
    'state',
    [
        _state(ControlAuthorityState.HOLD, 1, acknowledged=True),
        _state(ControlAuthorityState.AUTONOMY, 0, acknowledged=True),
        _state(ControlAuthorityState.HOLD, 0, acknowledged=False),
        _state(ControlAuthorityState.HOLD, 0, acknowledged=True, pending=1),
    ],
)
def test_cold_bootstrap_rejects_ambiguous_or_stale_state(state):
    with pytest.raises(RuntimeError):
        validate_cold_bootstrap_state(state)


def test_resumed_state_must_be_same_epoch_and_consume_ack():
    state = _state(
        ControlAuthorityState.AUTONOMY,
        1,
        acknowledged=False,
        source='autonomy',
    )

    assert validate_resumed_state(
        state,
        manager_epoch=41,
        previous_sequence=0,
    ) == 1


@pytest.mark.parametrize(
    'state',
    [
        _state(
            ControlAuthorityState.AUTONOMY,
            1,
            epoch=42,
            source='autonomy',
        ),
        _state(
            ControlAuthorityState.AUTONOMY,
            0,
            source='autonomy',
        ),
        _state(
            ControlAuthorityState.AUTONOMY,
            1,
            acknowledged=True,
            source='autonomy',
        ),
        _state(ControlAuthorityState.HOLD, 1),
    ],
)
def test_resumed_state_rejects_wrong_generation_or_wire_invariant(state):
    with pytest.raises(RuntimeError):
        validate_resumed_state(
            state,
            manager_epoch=41,
            previous_sequence=0,
        )
