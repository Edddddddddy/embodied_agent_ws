import threading
import time
from dataclasses import dataclass
from typing import Callable, Iterable, List

from .types import ActionCommand


@dataclass(frozen=True)
class SequencePublishReport:
    published: int
    completed: int
    failed: bool
    reason: str = ""


class SequentialActionPublisher:
    """按 ROS Action 结果节拍发布组合动作。

    Agent 只知道“发布候选动作”和“收到终态结果”这两个很小的接口；
    等待、超时、失败急停都封装在这里，避免 online/offline 节点各写一份状态机。
    """

    def __init__(self, result_timeout_s: float = 12.0):
        self.result_timeout_s = result_timeout_s
        self._condition = threading.Condition()
        self._result_count = 0
        self._last_result: dict | None = None
        self._results_by_id: dict[str, dict] = {}
        self._cancel_generation = 0
        self._cancel_reason = "cancelled"
        self._request_sequence = 0

    def notify_result(
        self,
        command_id: str,
        success: bool,
        message: str,
        *,
        status: int = 0,
    ) -> None:
        """接收强类型 Action 终态；无 command_id 的旧顺序兼容路径已删除。"""

        if not command_id:
            return
        result = {
            "command_id": command_id,
            "success": bool(success),
            "status": int(status),
            "message": message,
        }
        with self._condition:
            self._result_count += 1
            self._last_result = result
            self._results_by_id[command_id] = result
            self._condition.notify_all()

    def cancel(self, reason: str = "cancelled") -> None:
        """取消正在等待结果的组合动作。

        连续语音模式下，“停下/急停”必须抢占当前队列；如果这里不唤醒等待线程，
        原来的“走正方形/演示一下”可能会在 stop 之后继续发布下一步。
        """

        with self._condition:
            self._cancel_generation += 1
            self._cancel_reason = reason
            self._condition.notify_all()

    def publish(
        self,
        actions: Iterable[ActionCommand],
        publish_payload: Callable[[ActionCommand], None],
        *,
        wait_for_results: bool,
    ) -> SequencePublishReport:
        action_list = list(actions)
        if not action_list:
            return SequencePublishReport(0, 0, False)

        published = 0
        completed = 0
        with self._condition:
            generation = self._cancel_generation
        for action in action_list:
            with self._condition:
                if self._cancel_generation != generation:
                    return SequencePublishReport(
                        published, completed, True, self._cancel_reason
                    )
                command = self._command_with_request_id(action)
                request_id = command.request_id
            publish_payload(command)
            published += 1
            if not wait_for_results:
                continue
            success, reason = self._wait_for_result(request_id, generation)
            if success:
                completed += 1
                continue
            if reason == self._cancel_reason:
                return SequencePublishReport(published, completed, True, reason)
            publish_payload(self._command_with_request_id(ActionCommand("stop", {})))
            return SequencePublishReport(published + 1, completed, True, reason)
        return SequencePublishReport(published, completed, False)

    def _command_with_request_id(self, action: ActionCommand) -> ActionCommand:
        request_id = action.request_id
        if not request_id:
            self._request_sequence += 1
            request_id = f"agent-action-{self._request_sequence}"
        return ActionCommand(action.name, dict(action.arguments), request_id)

    def _wait_for_result(self, request_id: str, generation: int) -> tuple[bool, str]:
        deadline = time.monotonic() + max(0.0, self.result_timeout_s)
        with self._condition:
            while request_id not in self._results_by_id:
                if self._cancel_generation != generation:
                    return False, self._cancel_reason
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False, "action_result_timeout"
                self._condition.wait(timeout=remaining)
            if self._cancel_generation != generation:
                return False, self._cancel_reason
            result = self._results_by_id.pop(request_id)
        if result.get("success") is True:
            return True, str(result.get("message", "succeeded"))
        return False, str(result.get("message", "action_failed"))
