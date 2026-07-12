"""在线/离线 Agent 共用的命令执行并发运行时。"""

from __future__ import annotations

import queue
import threading
from typing import Callable

from .agent_control_plane import AgentControlPlane


class AgentExecutionRuntime:
    """拥有 busy 状态、连续队列 worker 与异常隔离。

    provider 只实现 ``execute_item``；本类保证 started/finished 事件成对、busy 最终复位，
    且单条失败不会杀死长时间语音控制 worker。
    """

    def __init__(
        self,
        control: AgentControlPlane,
        *,
        execute_item: Callable[[object], None],
        publish_execution: Callable[[object], None],
        publish_queue: Callable[[object], None],
        on_error: Callable[[Exception], None],
        before_execute: Callable[[object], None] | None = None,
    ):
        self._control = control
        self._execute_item = execute_item
        self._publish_execution = publish_execution
        self._publish_queue = publish_queue
        self._on_error = on_error
        self._before_execute = before_execute or (lambda _item: None)
        self._state_lock = threading.Lock()
        self._busy = False
        self._stop = threading.Event()
        self._worker: threading.Thread | None = None

    def start(self) -> None:
        if not self._control.continuous_enabled or self._worker is not None:
            return
        self._worker = threading.Thread(target=self._run_worker, daemon=True)
        self._worker.start()

    def try_begin_turn(self) -> bool:
        with self._state_lock:
            if self._busy or self._stop.is_set():
                return False
            self._busy = True
            return True

    def _begin_worker_turn(self) -> None:
        with self._state_lock:
            self._busy = True

    def finish_turn(self) -> None:
        with self._state_lock:
            self._busy = False

    def is_busy(self) -> bool:
        with self._state_lock:
            return self._busy

    def is_worker_thread(self) -> bool:
        return self._worker is not None and threading.current_thread() is self._worker

    def should_wait_for_action_results(self, action_count: int) -> bool:
        # 组合动作在所有模式都必须等结果；连续模式的单动作也必须串行完成。
        return action_count > 1 or (
            action_count > 0
            and self._control.continuous_enabled
            and self.is_worker_thread()
        )

    def _run_worker(self) -> None:
        while not self._stop.is_set():
            try:
                item = self._control.command_queue.get(
                    timeout=0.1,
                    on_stale=self._publish_expired,
                )
            except queue.Empty:
                continue

            if self._stop.is_set():
                self._control.command_queue.task_done()
                break

            self._begin_worker_turn()
            try:
                self._publish_execution(self._control.execution_started(item))
                self._before_execute(item)
                self._execute_item(item)
                self._publish_execution(
                    self._control.execution_finished(
                        item, success=True, reason="completed"
                    )
                )
            except Exception as exc:  # worker 必须隔离单条失败并继续消费后续命令。
                self._publish_execution(
                    self._control.execution_finished(
                        item, success=False, reason=str(exc)
                    )
                )
                self._on_error(exc)
            finally:
                self.finish_turn()
                self._control.command_queue.task_done()

    def _publish_expired(self, item) -> None:
        self._publish_queue(self._control.queue_expired(item))

    def stop(self, timeout_s: float = 1.0) -> None:
        self._stop.set()
        if self._worker is not None:
            self._worker.join(timeout=timeout_s)
