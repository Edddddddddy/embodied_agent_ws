"""ROS 无关的控制权 epoch、序号与租约校验。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


HOLD = 0
AUTONOMY = 1
KEYBOARD = 2
ESTOP = 3
_VALID_AUTHORITIES = {HOLD, AUTONOMY, KEYBOARD, ESTOP}


@dataclass(frozen=True, slots=True)
class AuthoritySnapshot:
    authority: int
    estop_latched: bool
    manager_epoch: int
    transition_sequence: int
    active_source: str
    reason: str
    pending_autonomy_revocation_sequence: int = 0
    # 旧消息缺字段时必须按“未确认”处理，不能用兼容默认值伪造恢复许可。
    autonomy_quiescence_acknowledged: bool = False


class AuthorityUpdateDecision(str, Enum):
    ACCEPTED_INITIAL = "accepted_initial"
    ACCEPTED_HEARTBEAT = "accepted_heartbeat"
    ACCEPTED_TRANSITION = "accepted_transition"
    ACCEPTED_MANAGER_RESTART = "accepted_manager_restart"
    REJECTED_INVALID_PAYLOAD = "invalid_payload"
    REJECTED_RETIRED_EPOCH = "retired_epoch"
    REJECTED_UNSAFE_EPOCH_BOOTSTRAP = "unsafe_epoch_bootstrap"
    REJECTED_LEASE_DISCONTINUITY = "lease_discontinuity"
    REJECTED_OLD_SEQUENCE = "old_sequence"
    REJECTED_CONFLICTING_HEARTBEAT = "conflicting_heartbeat"
    REJECTED_REORDERED_RECEIPT = "reordered_receipt"


@dataclass(frozen=True, slots=True)
class AuthorityUpdate:
    accepted: bool
    generation_changed: bool
    lease_discontinuity: bool
    decision: AuthorityUpdateDecision


class ControlAuthorityLease:
    """维护高层任务准入所需的本地安全代际。

    ROS 消息中的序号只在单个 manager epoch 内单调；本类额外维护本地
    generation，使 manager 重启或心跳中断前已排队的任务永久失效。
    """

    def __init__(self, lease_s: float = 1.0) -> None:
        if lease_s <= 0.0:
            raise ValueError("authority lease_s must be greater than zero")
        self._lease_s = float(lease_s)
        self._snapshot: AuthoritySnapshot | None = None
        self._received_at: float | None = None
        self._generation = 0
        self._retired_epochs: set[int] = set()

    @property
    def observed(self) -> bool:
        return self._snapshot is not None and self._received_at is not None

    @property
    def generation(self) -> int:
        return self._generation

    @property
    def authority(self) -> int:
        return HOLD if self._snapshot is None else self._snapshot.authority

    def fresh(self, now: float) -> bool:
        return (
            self._received_at is not None
            and now >= self._received_at
            and now - self._received_at <= self._lease_s
        )

    def fresh_autonomy(self, now: float) -> bool:
        return self.fresh(now) and self.authority == AUTONOMY

    def update(
        self, snapshot: AuthoritySnapshot, received_at: float
    ) -> AuthorityUpdate:
        if not self._payload_is_valid(snapshot):
            return self._reject(
                AuthorityUpdateDecision.REJECTED_INVALID_PAYLOAD
            )
        if self._snapshot is None or self._received_at is None:
            return self._accept(
                snapshot,
                received_at,
                AuthorityUpdateDecision.ACCEPTED_INITIAL,
                generation_changed=True,
            )
        if received_at < self._received_at:
            return self._reject(
                AuthorityUpdateDecision.REJECTED_REORDERED_RECEIPT
            )

        current = self._snapshot
        if snapshot.manager_epoch != current.manager_epoch:
            if snapshot.manager_epoch in self._retired_epochs:
                return self._reject(
                    AuthorityUpdateDecision.REJECTED_RETIRED_EPOCH
                )
            # 新进程必须先发 seq=0/HOLD；不允许旧进程的迟到 AUTONOMY
            # 通过“epoch 不同”伪装成一次 manager 重启。
            if not self._safe_manager_bootstrap(snapshot):
                return self._reject(
                    AuthorityUpdateDecision.REJECTED_UNSAFE_EPOCH_BOOTSTRAP
                )
            self._retired_epochs.add(current.manager_epoch)
            return self._accept(
                snapshot,
                received_at,
                AuthorityUpdateDecision.ACCEPTED_MANAGER_RESTART,
                generation_changed=True,
                lease_discontinuity=True,
            )

        # 与 C++ 最终速度 Gate 保持 sticky fail-closed：同一 manager epoch
        # 一旦断租，迟到心跳或更高 transition sequence 都不能恢复运动授权。
        # 只有新 epoch 的 seq=0/HOLD bootstrap 才能重新建立信任。
        if received_at - self._received_at > self._lease_s:
            return self._reject(
                AuthorityUpdateDecision.REJECTED_LEASE_DISCONTINUITY,
                lease_discontinuity=True,
            )

        if snapshot.transition_sequence < current.transition_sequence:
            return self._reject(
                AuthorityUpdateDecision.REJECTED_OLD_SEQUENCE
            )
        if snapshot.transition_sequence == current.transition_sequence:
            if snapshot != current:
                return self._reject(
                    AuthorityUpdateDecision.REJECTED_CONFLICTING_HEARTBEAT
                )
            self._received_at = received_at
            return AuthorityUpdate(
                True,
                False,
                False,
                AuthorityUpdateDecision.ACCEPTED_HEARTBEAT,
            )

        return self._accept(
            snapshot,
            received_at,
            AuthorityUpdateDecision.ACCEPTED_TRANSITION,
            generation_changed=True,
        )

    @staticmethod
    def _payload_is_valid(snapshot: AuthoritySnapshot) -> bool:
        if (
            snapshot.authority not in _VALID_AUTHORITIES
            or snapshot.manager_epoch <= 0
            or snapshot.transition_sequence < 0
            or snapshot.pending_autonomy_revocation_sequence < 0
            or snapshot.pending_autonomy_revocation_sequence
            > snapshot.transition_sequence
            or snapshot.estop_latched != (snapshot.authority == ESTOP)
        ):
            return False
        if snapshot.authority == AUTONOMY and (
            snapshot.pending_autonomy_revocation_sequence != 0
            or snapshot.autonomy_quiescence_acknowledged
        ):
            # 与 C++ ControlAuthorityTracker 使用同一 wire invariant：
            # ACK 在进入 AUTONOMY 时已被一次性消费，因此不得继续携带 pending/ack。
            return False
        if snapshot.authority == HOLD:
            return not snapshot.active_source
        return bool(snapshot.active_source)

    @staticmethod
    def _safe_manager_bootstrap(snapshot: AuthoritySnapshot) -> bool:
        return (
            snapshot.manager_epoch > 0
            and snapshot.transition_sequence == 0
            and snapshot.authority == HOLD
            and not snapshot.estop_latched
            and not snapshot.active_source
            and snapshot.reason == "initialized"
        )

    @staticmethod
    def _reject(
        decision: AuthorityUpdateDecision,
        *,
        lease_discontinuity: bool = False,
    ) -> AuthorityUpdate:
        return AuthorityUpdate(
            False,
            False,
            lease_discontinuity,
            decision,
        )

    def _accept(
        self,
        snapshot: AuthoritySnapshot,
        received_at: float,
        decision: AuthorityUpdateDecision,
        *,
        generation_changed: bool,
        lease_discontinuity: bool = False,
    ) -> AuthorityUpdate:
        self._snapshot = snapshot
        self._received_at = received_at
        if generation_changed:
            self._generation += 1
        return AuthorityUpdate(
            True,
            generation_changed,
            lease_discontinuity,
            decision,
        )
