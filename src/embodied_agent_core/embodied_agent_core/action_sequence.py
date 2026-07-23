import threading
import time
from dataclasses import dataclass
from typing import Callable, Iterable

from .types import ActionCommand


@dataclass(frozen=True)
class SequencePublishReport:
    published: int
    completed: int
    failed: bool
    reason: str = ""


class SequentialActionPublisher:
    """为组合动作分配 ID、批量发布，并按 ID 等待终态。

    真正的 FIFO、Action Client 和优先取消已经收敛到 C++ ActionScheduler；这里仅保留
    Agent 需要的批次完成语义，避免 Python 与 C++ 同时决定“下一条何时执行”。
    """

    _LONG_RUNNING_ACTIONS = frozenset({"navigate_to", "follow_waypoints"})

    def __init__(
        self,
        result_timeout_s: float = 12.0,
        long_action_result_timeout_s: float | None = None,
    ):
        self.result_timeout_s = result_timeout_s
        # move/turn 的终态通常在数秒内返回，而 Nav2 导航可能包含规划、恢复和多航点
        # 执行。二者共用 12 秒会让 Agent 误判导航超时并发布 STOP；分层超时既保留
        # 短动作故障的快速暴露，也允许长任务由底层 Action 正常返回终态。
        self.long_action_result_timeout_s = (
            result_timeout_s
            if long_action_result_timeout_s is None
            else long_action_result_timeout_s
        )
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
            commands = [self._command_with_request_id(action) for action in action_list]

        # 先把整个批次交给 C++ scheduler。即使后续某一步失败，Python 发出的 STOP
        # 也会让 scheduler 取消 active 并清空尚未执行的同批命令。
        for command in commands:
            with self._condition:
                if self._cancel_generation != generation:
                    return SequencePublishReport(
                        published, completed, True, self._cancel_reason
                    )
            publish_payload(command)
            published += 1
        if not wait_for_results:
            return SequencePublishReport(published, completed, False)

        for command in commands:
            success, reason = self._wait_for_result(
                command.request_id,
                generation,
                self._result_timeout_for(command),
            )
            if success:
                completed += 1
                continue
            if reason == self._cancel_reason:
                return SequencePublishReport(published, completed, True, reason)
            publish_payload(
                self._command_with_request_id(ActionCommand("stop", {}, priority=True))
            )
            return SequencePublishReport(published + 1, completed, True, reason)
        return SequencePublishReport(published, completed, False)

    def _command_with_request_id(self, action: ActionCommand) -> ActionCommand:
        request_id = action.request_id
        if not request_id:
            self._request_sequence += 1
            request_id = f"agent-action-{self._request_sequence}"
        return ActionCommand(
            action.name, dict(action.arguments), request_id, priority=action.priority
        )

    def _result_timeout_for(self, command: ActionCommand) -> float:
        if command.name in self._LONG_RUNNING_ACTIONS:
            return self.long_action_result_timeout_s
        return self.result_timeout_s

    def _wait_for_result(
        self,
        request_id: str,
        generation: int,
        timeout_s: float,
    ) -> tuple[bool, str]:
        deadline = time.monotonic() + max(0.0, timeout_s)
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
