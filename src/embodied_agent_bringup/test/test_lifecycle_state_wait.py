from __future__ import annotations

import pytest

from embodied_agent_bringup.lifecycle_state_wait import wait_for_state


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, duration_s: float) -> None:
        self.now += duration_s


def test_wait_for_state_tolerates_transitions_and_transient_read_errors() -> None:
    clock = FakeClock()
    observations = iter([1, RuntimeError("transition in progress"), 2, 3])

    def read_state() -> int:
        value = next(observations)
        if isinstance(value, Exception):
            raise value
        return value

    assert wait_for_state(
        read_state,
        expected_state=3,
        timeout_s=1.0,
        poll_interval_s=0.1,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    ) == 3


def test_wait_for_state_reports_last_observed_state_on_timeout() -> None:
    clock = FakeClock()

    with pytest.raises(TimeoutError, match="last_state=2"):
        wait_for_state(
            lambda: 2,
            expected_state=3,
            timeout_s=0.25,
            poll_interval_s=0.1,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        )
