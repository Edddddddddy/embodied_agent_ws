"""动态代价观测的锁存与报告字段适配。"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Protocol

from tools.acceptance.probes.slam_nav.session_observer import wait_until


class CostReader(Protocol):
    """只暴露动态证据所需的最小 costmap 查询接口。"""

    def cost_at(self, x: float, y: float) -> int: ...


@dataclass(frozen=True, slots=True)
class PredictedCostConfirmation:
    """一次等待窗口内已确认的动态代价快照。"""

    first_lethal_cost: int
    first_lethal_at_monotonic_s: float
    maximum_observed_cost: int
    maximum_observed_at_monotonic_s: float


class PredictedCostLatch:
    """锁存首次与最大 lethal 观测，避免稍后的 TTL 清理改写历史事实。"""

    def __init__(self, *, minimum_cost: int) -> None:
        if not 0 <= int(minimum_cost) <= 255:
            raise ValueError("minimum predicted cost must be within [0, 255]")
        self._minimum_cost = int(minimum_cost)
        self._first: tuple[int, float] | None = None
        self._maximum: tuple[int, float] | None = None

    def observe(self, cost: int, *, observed_at_monotonic_s: float) -> bool:
        rendered_cost = int(cost)
        rendered_time = float(observed_at_monotonic_s)
        if not math.isfinite(rendered_time):
            raise ValueError("predicted cost observation time must be finite")
        if rendered_cost < self._minimum_cost:
            return self._first is not None
        if self._first is None:
            self._first = (rendered_cost, rendered_time)
        if self._maximum is None or rendered_cost > self._maximum[0]:
            self._maximum = (rendered_cost, rendered_time)
        return True

    def confirmation(self) -> PredictedCostConfirmation:
        if self._first is None or self._maximum is None:
            raise RuntimeError("predicted lethal cost was not confirmed")
        return PredictedCostConfirmation(
            first_lethal_cost=self._first[0],
            first_lethal_at_monotonic_s=self._first[1],
            maximum_observed_cost=self._maximum[0],
            maximum_observed_at_monotonic_s=self._maximum[1],
        )


def confirm_predicted_lethal_cost(
    reader: CostReader,
    *,
    x: float,
    y: float,
    minimum_cost: int,
    timeout_s: float,
) -> PredictedCostConfirmation:
    """等待 predicted cell 达到门槛，并返回不可被后续衰减覆盖的证据。"""

    latch = PredictedCostLatch(minimum_cost=minimum_cost)

    def sample_cost() -> bool:
        return latch.observe(
            reader.cost_at(x, y),
            observed_at_monotonic_s=time.monotonic(),
        )

    wait_until(
        sample_cost,
        timeout_s,
        "predicted dynamic cost was not marked lethal",
    )
    return latch.confirmation()


def prediction_cost_evidence_fields(
    confirmation: PredictedCostConfirmation,
    *,
    route_commit_cost: int,
) -> dict[str, int | float]:
    """保持旧 ``cost`` 字段语义，并附加规划提交时的衰减诊断。"""

    return {
        # cost 是 evaluator 的兼容字段，必须表达“门槛曾被真实观察到”；
        # route commit 时的即时值可能已因 0.8s TTL 正常下降，不能倒写历史。
        "cost": confirmation.maximum_observed_cost,
        "confirmed_cost": confirmation.maximum_observed_cost,
        "confirmed_at_monotonic_s": (
            confirmation.maximum_observed_at_monotonic_s
        ),
        "first_lethal_cost": confirmation.first_lethal_cost,
        "first_lethal_at_monotonic_s": (
            confirmation.first_lethal_at_monotonic_s
        ),
        "maximum_observed_cost": confirmation.maximum_observed_cost,
        "maximum_observed_at_monotonic_s": (
            confirmation.maximum_observed_at_monotonic_s
        ),
        "route_commit_cost": int(route_commit_cost),
    }
