"""把文本动作请求与异步 typed Action 结果可靠关联。"""

from __future__ import annotations

from collections import OrderedDict, deque
from dataclasses import dataclass
import threading
import time
from typing import Callable

from .mission_executor import AutomaticMissionCancelled, CommandRequest


@dataclass(frozen=True, slots=True)
class AgentActionOutcome:
    success: bool
    status: int
    message: str


class AgentActionGateway:
    """串行执行 Agent 文本动作，并屏蔽 ROS 回调的并发细节。"""

    def __init__(
        self,
        *,
        publish_text: Callable[[str], None],
        subscriber_count: Callable[[], int],
        cancel_motion: Callable[[], None],
        dry_run: bool = False,
        log_dry_run: Callable[[str], None] = lambda _message: None,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._publish_text = publish_text
        self._subscriber_count = subscriber_count
        self._cancel_motion = cancel_motion
        self._dry_run = dry_run
        self._log_dry_run = log_dry_run
        self._clock = clock
        self._sleep = sleep
        self._run_lock = threading.Lock()
        self._condition = threading.Condition()
        self._candidate_generation = 0
        self._candidates: deque[tuple[int, int, str]] = deque(maxlen=64)
        self._result_generation = 0
        self._results: OrderedDict[
            str, tuple[int, AgentActionOutcome]
        ] = OrderedDict()

    def record_candidate(self, action_type: int, command_id: str) -> None:
        if not command_id:
            return
        with self._condition:
            self._candidate_generation += 1
            self._candidates.append(
                (self._candidate_generation, int(action_type), command_id)
            )
            self._condition.notify_all()

    def record_result(
        self, command_id: str, outcome: AgentActionOutcome
    ) -> None:
        if not command_id:
            return
        with self._condition:
            self._result_generation += 1
            self._results[command_id] = (self._result_generation, outcome)
            self._results.move_to_end(command_id)
            while len(self._results) > 128:
                self._results.popitem(last=False)
            self._condition.notify_all()

    def run(
        self,
        request: CommandRequest,
        *,
        text: str,
        expected_action_type: int,
        expected_action_name: str,
        timeout_s: float,
    ) -> AgentActionOutcome | None:
        if self._dry_run:
            self._log_dry_run(
                f"DRY RUN agent action: {text} -> {expected_action_name}"
            )
            return None
        with self._run_lock:
            return self._run_once(
                request,
                text=text,
                expected_action_type=expected_action_type,
                expected_action_name=expected_action_name,
                timeout_s=timeout_s,
            )

    def _run_once(
        self,
        request: CommandRequest,
        *,
        text: str,
        expected_action_type: int,
        expected_action_name: str,
        timeout_s: float,
    ) -> AgentActionOutcome:
        deadline = self._clock() + max(0.0, timeout_s)
        while self._subscriber_count() == 0:
            self._raise_if_canceled(request)
            if self._clock() >= deadline:
                raise TimeoutError("Agent text input subscriber unavailable")
            self._sleep(min(0.1, max(0.0, deadline - self._clock())))

        with self._condition:
            candidate_baseline = self._candidate_generation
            result_baseline = self._result_generation
        self._publish_text(text)
        command_id = self._wait_for_candidate(
            request, candidate_baseline, expected_action_type, deadline
        )
        outcome = self._wait_for_result(
            request, command_id, result_baseline, deadline, expected_action_name
        )
        if not outcome.success:
            raise RuntimeError(
                f"{expected_action_name} failed status={outcome.status}: "
                f"{outcome.message}"
            )
        return outcome

    def _wait_for_candidate(
        self, request, baseline: int, expected_type: int, deadline: float
    ) -> str:
        with self._condition:
            while True:
                self._raise_if_canceled(request)
                for generation, action_type, command_id in self._candidates:
                    if generation > baseline and action_type == expected_type:
                        return command_id
                remaining = deadline - self._clock()
                if remaining <= 0.0:
                    raise TimeoutError("Agent did not publish expected action")
                self._condition.wait(timeout=min(0.2, remaining))

    def _wait_for_result(
        self, request, command_id, baseline, deadline, expected_name
    ) -> AgentActionOutcome:
        with self._condition:
            while True:
                self._raise_if_canceled(request)
                recorded = self._results.get(command_id)
                # Agent 重启后 command_id 可能从 1 重新计数；回调代次防止旧结果
                # 误完成当前任务，这是只比较字符串 ID 做不到的关联保证。
                if recorded is not None and recorded[0] > baseline:
                    self._results.pop(command_id, None)
                    return recorded[1]
                remaining = deadline - self._clock()
                if remaining <= 0.0:
                    raise TimeoutError(
                        f"action result timeout for {expected_name}:{command_id}"
                    )
                self._condition.wait(timeout=min(0.2, remaining))

    def _raise_if_canceled(self, request: CommandRequest) -> None:
        if not request.canceled:
            return
        # 急停先旁路普通动作队列，再把取消传播给高层事务。
        self._cancel_motion()
        raise AutomaticMissionCancelled("automatic mission canceled")
