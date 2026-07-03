import json
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
        self._cancel_generation = 0
        self._cancel_reason = "cancelled"

    def notify_result(self, serialized_result: str) -> None:
        try:
            result = json.loads(serialized_result)
        except json.JSONDecodeError:
            return
        if not isinstance(result, dict) or "success" not in result:
            return
        with self._condition:
            self._result_count += 1
            self._last_result = result
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
        publish_payload: Callable[[str], None],
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
                previous_count = self._result_count
                if self._cancel_generation != generation:
                    return SequencePublishReport(
                        published, completed, True, self._cancel_reason
                    )
            publish_payload(json.dumps(action.as_dict(), ensure_ascii=False))
            published += 1
            if not wait_for_results:
                continue
            success, reason = self._wait_for_next_result(previous_count, generation)
            if success:
                completed += 1
                continue
            if reason == self._cancel_reason:
                return SequencePublishReport(published, completed, True, reason)
            publish_payload(json.dumps(ActionCommand("stop", {}).as_dict(), ensure_ascii=False))
            return SequencePublishReport(published + 1, completed, True, reason)
        return SequencePublishReport(published, completed, False)

    def _wait_for_next_result(self, previous_count: int, generation: int) -> tuple[bool, str]:
        deadline = time.monotonic() + max(0.0, self.result_timeout_s)
        with self._condition:
            while self._result_count <= previous_count:
                if self._cancel_generation != generation:
                    return False, self._cancel_reason
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False, "action_result_timeout"
                self._condition.wait(timeout=remaining)
            if self._cancel_generation != generation:
                return False, self._cancel_reason
            result = self._last_result or {}
        if result.get("success") is True:
            return True, str(result.get("message", "succeeded"))
        return False, str(result.get("message", "action_failed"))
