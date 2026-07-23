"""控制权撤销后的自治静默屏障；本模块不依赖 ROS。"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
import math
import threading
import time
from typing import Callable


class QuiescenceState(str, Enum):
    IDLE = "idle"
    WAITING = "waiting"
    READY = "ready"
    ACK_IN_FLIGHT = "ack_in_flight"
    ACKNOWLEDGED = "acknowledged"
    FAILED = "failed"


class QuiescenceError(RuntimeError):
    """屏障已失败或调用顺序违反静默协议。"""


class QuiescenceTimeout(QuiescenceError):
    """在安全预算内没有收齐全部可审计终态。"""


@dataclass(frozen=True, slots=True)
class QuiescenceIdentity:
    """一次 AUTONOMY 撤销的稳定身份，不使用会继续增长的当前迁移序号。"""

    manager_epoch: int
    revocation_sequence: int

    def __post_init__(self) -> None:
        if self.manager_epoch <= 0:
            raise ValueError("manager_epoch must be positive")
        if self.revocation_sequence < 0:
            raise ValueError("revocation_sequence must be non-negative")


@dataclass(frozen=True, slots=True)
class QuiescenceSnapshot:
    state: QuiescenceState
    identity: QuiescenceIdentity | None
    ready: bool
    missing: tuple[str, ...]
    failure_reason: str
    frontier_required: bool
    nav2_goal_count: int
    stop_command_id: str
    fresh_zero_generation: int | None


class AutonomyQuiescenceBarrier:
    """把多条异步证据收口成一个可 ACK 的安全事实。

    Interface 只有“记录事实、开始撤销、等待、确认”四类操作。ROS topic、
    Action future 和 service future 都留在 Node adapter 中；因此并发协议可在
    不启动 DDS/Gazebo 的情况下被完整测试。
    """

    _FRONTIER_ACTIVE_STATUSES = frozenset(
        {"exploration_started", "exploration_running"}
    )
    _FRONTIER_TERMINAL_STATUSES = frozenset(
        {
            "exploration_paused",
            "exploration_complete",
            "exploration_completed",
            "exploration_stopped",
        }
    )

    def __init__(
        self,
        *,
        zero_epsilon: float = 1.0e-3,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not math.isfinite(zero_epsilon) or zero_epsilon < 0.0:
            raise ValueError("zero_epsilon must be finite and non-negative")
        self._zero_epsilon = float(zero_epsilon)
        self._clock = clock
        self._condition = threading.Condition(threading.RLock())
        self._state = QuiescenceState.IDLE
        self._identity: QuiescenceIdentity | None = None
        self._observed_authority: QuiescenceIdentity | None = None
        self._failure_reason = ""

        self._frontier_active = False
        self._frontier_terminal = True
        self._frontier_required = False
        self._active_nav2_goals: set[str] = set()
        self._required_nav2_goals: set[str] = set()
        self._terminal_nav2_goals: set[str] = set()
        self._terminal_nav2_goal_order: deque[str] = deque()

        self._stop_command_id = ""
        self._stop_result_observed = False
        self._stop_cmd_vel_generation: int | None = None
        self._fresh_zero_generation: int | None = None
        self._acknowledged_identities: set[QuiescenceIdentity] = set()
        self._acknowledged_identity_order: deque[QuiescenceIdentity] = deque()

    @property
    def active(self) -> bool:
        with self._condition:
            return self._state in {
                QuiescenceState.WAITING,
                QuiescenceState.READY,
                QuiescenceState.ACK_IN_FLIGHT,
            }

    @property
    def identity(self) -> QuiescenceIdentity | None:
        with self._condition:
            return self._identity

    def observe_authority(
        self,
        identity: QuiescenceIdentity,
        *,
        autonomy_active: bool = False,
    ) -> None:
        """记录 manager 状态，并容纳 ACK 后的合法快速恢复。

        manager 只能在接受本轮 ACK 后进入 AUTONOMY。因此 service response 与
        state topic 被不同 executor 线程调度时，AUTONOMY 回调本身就是 ACK 已被
        manager 接受的权威事实。若 ACK 尚未发起，则任何恢复仍按协议错误失败。
        """

        with self._condition:
            active = self._state in {
                QuiescenceState.WAITING,
                QuiescenceState.READY,
                QuiescenceState.ACK_IN_FLIGHT,
            }
            if not active:
                self._observed_authority = identity
                self._condition.notify_all()
                return

            expected = self._identity
            if expected is None:
                self._fail_unlocked(
                    "active autonomy quiescence has no identity"
                )
                self._condition.notify_all()
                return

            if autonomy_active:
                if identity.manager_epoch != expected.manager_epoch:
                    self._fail_unlocked(
                        "authority manager epoch changed while quiescing: "
                        f"expected={expected.manager_epoch} "
                        f"observed={identity.manager_epoch}"
                    )
                elif self._state is QuiescenceState.ACK_IN_FLIGHT:
                    self._acknowledge_current_unlocked()
                else:
                    self._fail_unlocked(
                        "manager resumed autonomy before quiescence ACK started"
                    )
                self._observed_authority = identity
                self._condition.notify_all()
                return

            if identity != expected:
                if (
                    self._state is QuiescenceState.ACK_IN_FLIGHT
                    and identity.manager_epoch == expected.manager_epoch
                    and identity.revocation_sequence
                    > expected.revocation_sequence
                ):
                    # callback 可能跳过中间 AUTONOMY 心跳，直接看到下一次撤销。
                    # manager 状态机只有在接受旧 ACK 并恢复后才能生成更大的
                    # pending revocation，因此先幂等完成旧代，再交给 begin()。
                    self._acknowledge_current_unlocked()
                    self._observed_authority = identity
                else:
                    self._fail_unlocked(
                        "authority identity changed while quiescing: "
                        f"expected={expected} observed={identity}"
                    )
            else:
                self._observed_authority = identity
            self._condition.notify_all()

    def begin(
        self,
        identity: QuiescenceIdentity,
        *,
        require_frontier: bool | None = None,
    ) -> bool:
        """开始一次屏障；相同身份重复通知是幂等的。"""

        with self._condition:
            if self._state is QuiescenceState.FAILED:
                raise QuiescenceError(self._failure_reason)
            if self._state in {
                QuiescenceState.WAITING,
                QuiescenceState.READY,
                QuiescenceState.ACK_IN_FLIGHT,
            }:
                if identity == self._identity:
                    return False
                self._fail_unlocked(
                    "new autonomy revocation arrived before previous ACK"
                )
                raise QuiescenceError(self._failure_reason)
            if self._observed_authority != identity:
                raise QuiescenceError(
                    "cannot begin without matching authority observation"
                )

            self._identity = identity
            self._state = QuiescenceState.WAITING
            self._failure_reason = ""
            self._frontier_required = (
                self._frontier_active
                if require_frontier is None
                else bool(require_frontier) or self._frontier_active
            )
            self._required_nav2_goals = set(self._active_nav2_goals)
            self._stop_command_id = ""
            self._stop_result_observed = False
            self._stop_cmd_vel_generation = None
            self._fresh_zero_generation = None
            self._refresh_ready_unlocked()
            self._condition.notify_all()
            return True

    def observe_frontier(
        self,
        *,
        status: str,
        active_goal_count: int,
        accepted_goal_count: int,
        terminal_goal_count: int,
    ) -> None:
        """记录 Explore owner ledger；仅 active=0 不足以证明所有 UUID 结算。"""

        counters = (
            active_goal_count,
            accepted_goal_count,
            terminal_goal_count,
        )
        if any(value < 0 for value in counters):
            raise ValueError("frontier ledger counters must be non-negative")
        normalized_status = str(status).strip().lower()
        terminal = (
            normalized_status in self._FRONTIER_TERMINAL_STATUSES
            and active_goal_count == 0
            and accepted_goal_count == terminal_goal_count
        )
        active = (
            normalized_status in self._FRONTIER_ACTIVE_STATUSES
            or active_goal_count > 0
            or accepted_goal_count > terminal_goal_count
        )
        with self._condition:
            self._frontier_active = active
            self._frontier_terminal = terminal
            if (
                self._state
                in {
                    QuiescenceState.WAITING,
                    QuiescenceState.READY,
                    QuiescenceState.ACK_IN_FLIGHT,
                }
                and self._frontier_required
            ):
                self._refresh_ready_unlocked()
            self._condition.notify_all()

    def nav2_goal_started(self, token: str) -> None:
        normalized = str(token).strip()
        if not normalized:
            raise ValueError("Nav2 goal token must not be empty")
        with self._condition:
            if normalized in self._active_nav2_goals:
                raise QuiescenceError(f"duplicate active Nav2 goal: {normalized}")
            if normalized in self._terminal_nav2_goals:
                raise QuiescenceError(
                    f"reused terminal Nav2 goal token: {normalized}"
                )
            self._active_nav2_goals.add(normalized)
            if self._state in {
                QuiescenceState.WAITING,
                QuiescenceState.READY,
                QuiescenceState.ACK_IN_FLIGHT,
            }:
                # 撤销边沿之后出现新 goal 属于幽灵复驶；即便它很快 canceled，
                # 也不能把这次竞态解释为“最终静默”并向 manager ACK。
                self._fail_unlocked(
                    f"Nav2 goal started after autonomy revocation: {normalized}"
                )
            self._condition.notify_all()

    def nav2_goal_terminal(self, token: str) -> bool:
        normalized = str(token).strip()
        if not normalized:
            raise ValueError("Nav2 goal token must not be empty")
        with self._condition:
            if normalized not in self._active_nav2_goals:
                category = (
                    "duplicate"
                    if normalized in self._terminal_nav2_goals
                    else "unknown"
                )
                self._fail_unlocked(
                    f"{category} Nav2 terminal token: {normalized}"
                )
                self._condition.notify_all()
                raise QuiescenceError(self._failure_reason)
            self._active_nav2_goals.remove(normalized)
            self._required_nav2_goals.discard(normalized)
            self._remember_terminal_nav2_goal_unlocked(normalized)
            self._refresh_ready_unlocked()
            self._condition.notify_all()
            return True

    def mark_priority_stop_requested(
        self,
        command_id: str,
        *,
        cmd_vel_generation: int,
    ) -> None:
        normalized = str(command_id).strip()
        if not normalized:
            raise ValueError("priority STOP command_id must not be empty")
        if cmd_vel_generation < 0:
            raise ValueError("cmd_vel_generation must be non-negative")
        with self._condition:
            self._require_active_unlocked()
            if self._stop_command_id:
                if (
                    self._stop_command_id == normalized
                    and self._stop_cmd_vel_generation == cmd_vel_generation
                ):
                    return
                self._fail_unlocked(
                    "priority STOP request changed while quiescing"
                )
                raise QuiescenceError(self._failure_reason)
            self._stop_command_id = normalized
            self._stop_cmd_vel_generation = int(cmd_vel_generation)
            self._fresh_zero_generation = None
            self._refresh_ready_unlocked()
            self._condition.notify_all()

    def observe_priority_stop_result(
        self,
        command_id: str,
        *,
        success: bool,
        detail: str = "",
    ) -> bool:
        normalized = str(command_id).strip()
        with self._condition:
            if (
                self._state
                not in {
                    QuiescenceState.WAITING,
                    QuiescenceState.READY,
                    QuiescenceState.ACK_IN_FLIGHT,
                }
                or normalized != self._stop_command_id
            ):
                return False
            if not success:
                self._fail_unlocked(
                    "priority STOP failed"
                    + (f": {detail}" if detail else "")
                )
            else:
                self._stop_result_observed = True
                self._refresh_ready_unlocked()
            self._condition.notify_all()
            return True

    def observe_cmd_vel(
        self,
        *,
        generation: int,
        linear_x: float,
        angular_z: float,
    ) -> None:
        if generation < 0:
            raise ValueError("cmd_vel generation must be non-negative")
        if not math.isfinite(linear_x) or not math.isfinite(angular_z):
            raise ValueError("cmd_vel values must be finite")
        with self._condition:
            if (
                self._state
                not in {
                    QuiescenceState.WAITING,
                    QuiescenceState.READY,
                    QuiescenceState.ACK_IN_FLIGHT,
                }
                or self._stop_cmd_vel_generation is None
                or generation <= self._stop_cmd_vel_generation
            ):
                return
            if (
                abs(linear_x) <= self._zero_epsilon
                and abs(angular_z) <= self._zero_epsilon
            ):
                self._fresh_zero_generation = generation
            else:
                # 零速之后若又出现运动，之前证据立即失效；必须再收到更新一代零速。
                self._fresh_zero_generation = None
            self._refresh_ready_unlocked()
            self._condition.notify_all()

    def wait_ready(self, *, timeout_s: float) -> QuiescenceSnapshot:
        if not math.isfinite(timeout_s) or timeout_s < 0.0:
            raise ValueError("quiescence timeout must be finite and non-negative")
        deadline = self._clock() + timeout_s
        with self._condition:
            while True:
                if self._state is QuiescenceState.FAILED:
                    raise QuiescenceError(self._failure_reason)
                self._refresh_ready_unlocked()
                if self._state is QuiescenceState.READY:
                    return self._snapshot_unlocked()
                remaining = deadline - self._clock()
                if remaining <= 0.0:
                    missing = ",".join(self._missing_unlocked())
                    self._fail_unlocked(
                        f"autonomy quiescence timeout missing={missing}"
                    )
                    raise QuiescenceTimeout(self._failure_reason)
                self._condition.wait(timeout=remaining)

    def acknowledged(self, identity: QuiescenceIdentity) -> bool:
        with self._condition:
            return identity in self._acknowledged_identities

    def wait_acknowledged(
        self,
        identity: QuiescenceIdentity,
        *,
        timeout_s: float,
    ) -> QuiescenceSnapshot:
        """等待 exact revocation 完成，供上层保持任务互斥直到安全收口。"""

        if not math.isfinite(timeout_s) or timeout_s < 0.0:
            raise ValueError("quiescence timeout must be finite and non-negative")
        deadline = self._clock() + timeout_s
        with self._condition:
            while True:
                if identity in self._acknowledged_identities:
                    return self._snapshot_unlocked()
                if self._state is QuiescenceState.FAILED:
                    raise QuiescenceError(self._failure_reason)
                remaining = deadline - self._clock()
                if remaining <= 0.0:
                    raise QuiescenceTimeout(
                        "autonomy quiescence acknowledgement timeout "
                        f"identity={identity}"
                    )
                self._condition.wait(timeout=remaining)

    def begin_acknowledgement(
        self, identity: QuiescenceIdentity
    ) -> QuiescenceSnapshot:
        """冻结 READY 证据并进入显式 ACK 事务。"""

        with self._condition:
            if self._state is QuiescenceState.FAILED:
                raise QuiescenceError(self._failure_reason)
            self._refresh_ready_unlocked()
            if (
                self._state is not QuiescenceState.READY
                or identity != self._identity
                or identity != self._observed_authority
            ):
                raise QuiescenceError(
                    "cannot start acknowledgement for incomplete or stale "
                    "quiescence"
                )
            self._state = QuiescenceState.ACK_IN_FLIGHT
            self._condition.notify_all()
            return self._snapshot_unlocked()

    def complete_acknowledgement(
        self, identity: QuiescenceIdentity
    ) -> bool:
        """提交 ACK；已由权威 AUTONOMY 回调完成时保持幂等。"""

        with self._condition:
            if identity in self._acknowledged_identities:
                return False
            if self._state is QuiescenceState.FAILED:
                raise QuiescenceError(self._failure_reason)
            if (
                self._state is not QuiescenceState.ACK_IN_FLIGHT
                or identity != self._identity
                or identity != self._observed_authority
                or self._missing_unlocked()
            ):
                raise QuiescenceError(
                    "cannot complete acknowledgement for incomplete or stale "
                    "quiescence"
                )
            self._acknowledge_current_unlocked()
            self._condition.notify_all()
            return True

    def mark_acknowledged(self, identity: QuiescenceIdentity) -> None:
        """旧调用的兼容入口；新代码应显式 begin/complete ACK。"""

        self.begin_acknowledgement(identity)
        self.complete_acknowledgement(identity)

    def fail(self, reason: str) -> None:
        with self._condition:
            self._fail_unlocked(str(reason) or "quiescence failed")
            self._condition.notify_all()

    def snapshot(self) -> QuiescenceSnapshot:
        with self._condition:
            self._refresh_ready_unlocked()
            return self._snapshot_unlocked()

    def _snapshot_unlocked(self) -> QuiescenceSnapshot:
        missing = self._missing_unlocked()
        return QuiescenceSnapshot(
            state=self._state,
            identity=self._identity,
            ready=(
            self._state is QuiescenceState.READY and not missing
            ),
            missing=missing,
            failure_reason=self._failure_reason,
            frontier_required=self._frontier_required,
            nav2_goal_count=len(self._required_nav2_goals),
            stop_command_id=self._stop_command_id,
            fresh_zero_generation=self._fresh_zero_generation,
        )

    def _missing_unlocked(self) -> tuple[str, ...]:
        missing: list[str] = []
        if self._frontier_required and not self._frontier_terminal:
            missing.append("explore_ledger_terminal")
        if self._required_nav2_goals:
            missing.append("nav2_action_terminal")
        if not self._stop_result_observed:
            missing.append("priority_stop_result")
        if self._fresh_zero_generation is None:
            missing.append("fresh_zero_cmd_vel")
        return tuple(missing)

    def _refresh_ready_unlocked(self) -> None:
        if self._state is QuiescenceState.ACK_IN_FLIGHT:
            missing = self._missing_unlocked()
            if missing:
                self._fail_unlocked(
                    "quiescence evidence invalidated while ACK in flight: "
                    + ",".join(missing)
                )
            return
        if self._state not in {
            QuiescenceState.WAITING,
            QuiescenceState.READY,
        }:
            return
        self._state = (
            QuiescenceState.READY
            if not self._missing_unlocked()
            else QuiescenceState.WAITING
        )

    def _require_active_unlocked(self) -> None:
        if self._state is QuiescenceState.FAILED:
            raise QuiescenceError(self._failure_reason)
        if self._state not in {
            QuiescenceState.WAITING,
            QuiescenceState.READY,
            QuiescenceState.ACK_IN_FLIGHT,
        }:
            raise QuiescenceError("autonomy quiescence is not active")

    def _acknowledge_current_unlocked(self) -> None:
        identity = self._identity
        if identity is None:
            self._fail_unlocked("cannot acknowledge quiescence without identity")
            raise QuiescenceError(self._failure_reason)
        self._remember_acknowledged_identity_unlocked(identity)
        self._state = QuiescenceState.ACKNOWLEDGED

    def _remember_acknowledged_identity_unlocked(
        self, identity: QuiescenceIdentity
    ) -> None:
        if identity in self._acknowledged_identities:
            return
        if len(self._acknowledged_identity_order) >= 32:
            expired = self._acknowledged_identity_order.popleft()
            self._acknowledged_identities.discard(expired)
        self._acknowledged_identity_order.append(identity)
        self._acknowledged_identities.add(identity)

    def _remember_terminal_nav2_goal_unlocked(self, token: str) -> None:
        if len(self._terminal_nav2_goal_order) >= 128:
            expired = self._terminal_nav2_goal_order.popleft()
            self._terminal_nav2_goals.discard(expired)
        self._terminal_nav2_goal_order.append(token)
        self._terminal_nav2_goals.add(token)

    def _fail_unlocked(self, reason: str) -> None:
        if self._state is QuiescenceState.FAILED:
            return
        self._state = QuiescenceState.FAILED
        self._failure_reason = reason
