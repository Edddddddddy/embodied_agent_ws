"""在严格冷启动证明后，通过 typed service 恢复会话自治控制权."""

from __future__ import annotations

import time
from typing import Any, Callable

from embodied_agent_interfaces.msg import ControlAuthorityState
from embodied_agent_interfaces.srv import SetControlAuthority
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def validate_cold_bootstrap_state(state: Any) -> tuple[int, int]:
    """验证 manager 的初始 HOLD 确实携带本次冷启动许可."""
    epoch = int(state.manager_epoch)
    sequence = int(state.transition_sequence)
    if (
        epoch <= 0
        or sequence != 0
        or int(state.authority) != ControlAuthorityState.HOLD
        or bool(state.estop_latched)
        or int(state.pending_autonomy_revocation_sequence) != 0
        or not bool(state.autonomy_quiescence_acknowledged)
        or str(state.active_source)
    ):
        raise RuntimeError(
            'manager did not publish a safe cold-bootstrap HOLD state'
        )
    return epoch, sequence


def validate_resumed_state(
    state: Any,
    *,
    manager_epoch: int,
    previous_sequence: int,
) -> int:
    """锁住 RESUME 后的 wire invariant，避免接受旧 epoch 或宽松响应."""
    sequence = int(state.transition_sequence)
    if (
        int(state.manager_epoch) != manager_epoch
        or sequence <= previous_sequence
        or int(state.authority) != ControlAuthorityState.AUTONOMY
        or bool(state.estop_latched)
        or int(state.pending_autonomy_revocation_sequence) != 0
        or bool(state.autonomy_quiescence_acknowledged)
        or str(state.active_source) != 'autonomy'
    ):
        raise RuntimeError(
            'RESUME_AUTONOMY returned an invalid or stale authority state'
        )
    return sequence


class ColdStartAuthorityBootstrap(Node):
    """只消费一次冷启动许可，不承担后续控制权状态机职责."""

    def __init__(self) -> None:
        super().__init__('cold_start_authority_bootstrap')
        self._state: ControlAuthorityState | None = None
        self.create_subscription(
            ControlAuthorityState,
            '/control/authority/state',
            self._on_state,
            STATE_QOS,
        )
        self._client = self.create_client(
            SetControlAuthority,
            '/control/set_authority',
        )

    def _on_state(self, message: ControlAuthorityState) -> None:
        self._state = message

    def _wait_for_state(
        self,
        predicate: Callable[[ControlAuthorityState], bool],
        *,
        timeout_s: float,
    ) -> ControlAuthorityState:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._state is not None and predicate(self._state):
                return self._state
        raise TimeoutError('timed out waiting for control authority state')

    def resume(self, *, timeout_s: float = 6.0) -> tuple[int, int]:
        if not self._client.wait_for_service(timeout_sec=timeout_s):
            raise TimeoutError('/control/set_authority is unavailable')

        initial = self._wait_for_state(
            lambda _state: True,
            timeout_s=timeout_s,
        )
        manager_epoch, initial_sequence = validate_cold_bootstrap_state(
            initial
        )

        request = SetControlAuthority.Request()
        request.command = SetControlAuthority.Request.RESUME_AUTONOMY
        request.requester = 'showcase_session_root'
        request.reason = 'verified_cold_start'
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        response = future.result()
        if response is None or not bool(response.accepted):
            detail = 'no response' if response is None else response.message
            raise RuntimeError(f'RESUME_AUTONOMY rejected: {detail}')
        resumed_sequence = validate_resumed_state(
            response.state,
            manager_epoch=manager_epoch,
            previous_sequence=initial_sequence,
        )

        # service response 与 transient-local topic 属于不同 DDS 通道；等同代状态
        # 真正可见后再启动任务编排，避免 admission 读到旧 HOLD。
        self._wait_for_state(
            lambda state: (
                int(state.manager_epoch) == manager_epoch
                and int(state.transition_sequence) >= resumed_sequence
                and int(state.authority)
                == ControlAuthorityState.AUTONOMY
            ),
            timeout_s=timeout_s,
        )
        return manager_epoch, resumed_sequence


def main(args=None) -> int:
    rclpy.init(args=args)
    node = ColdStartAuthorityBootstrap()
    try:
        epoch, sequence = node.resume()
        print(
            'PASS: cold-start control authority resumed '
            f'epoch={epoch} sequence={sequence}',
            flush=True,
        )
        return 0
    except Exception as error:
        node.get_logger().error(str(error))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
