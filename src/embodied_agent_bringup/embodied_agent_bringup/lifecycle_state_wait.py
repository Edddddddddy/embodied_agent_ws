"""Small, ROS-independent polling policy for lifecycle state probes."""

from __future__ import annotations

import time
from collections.abc import Callable


def wait_for_state(
    read_state: Callable[[], int],
    *,
    expected_state: int,
    timeout_s: float,
    poll_interval_s: float = 0.2,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> int:
    """Poll a lifecycle service until the expected state is observed.

    A lifecycle transition briefly makes service calls fail or return an intermediate
    state.  Those observations are expected and must not turn startup into a flaky
    log-string race.
    """

    timeout_s = max(0.0, float(timeout_s))
    poll_interval_s = max(0.01, float(poll_interval_s))
    deadline = monotonic() + timeout_s
    last_state: int | None = None
    last_error: Exception | None = None

    while True:
        try:
            last_state = int(read_state())
            last_error = None
            if last_state == expected_state:
                return last_state
        except Exception as exc:  # transition 期间的短暂 service 错误允许重试
            last_error = exc

        remaining_s = deadline - monotonic()
        if remaining_s <= 0.0:
            details = f"last_state={last_state}"
            if last_error is not None:
                details += f", last_error={last_error}"
            raise TimeoutError(
                f"lifecycle state {expected_state} was not observed; {details}"
            )
        sleep(min(poll_interval_s, remaining_s))
