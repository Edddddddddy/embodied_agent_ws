"""在线/离线 Agent 共用的命令执行并发运行时。"""

from __future__ import annotations

import queue
import threading
import time
from typing import Callable

from .agent_control_plane import AgentControlPlane


class AgentExecutionCancelled(RuntimeError):
    """Lifecycle 停用正在执行的 turn；属于受控取消而不是 provider 故障。"""


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
        self._background_threads: set[threading.Thread] = set()

    def start(self) -> bool:
        """启动或重新启动连续 worker；仍有旧 turn 未退出时拒绝重入。"""

        with self._state_lock:
            self._discard_finished_threads_locked()
            background_alive = any(
                thread.is_alive() for thread in self._background_threads
            )
            worker_alive = self._worker is not None and self._worker.is_alive()
            if self._stop.is_set() and (background_alive or worker_alive):
                return False
            if background_alive:
                return False
            if worker_alive:
                return True
            self._stop.clear()
            if not self._control.continuous_enabled:
                return True
            self._worker = threading.Thread(target=self._run_worker, daemon=True)
            self._worker.start()
        return True

    def start_background_turn(self, execute: Callable, *args) -> bool:
        """让运行时拥有非连续 turn 的线程、busy 复位与停用等待语义。"""

        if not self.try_begin_turn():
            return False

        thread: threading.Thread

        def run() -> None:
            try:
                execute(*args)
            except AgentExecutionCancelled:
                pass
            except Exception as exc:
                self._on_error(exc)
            finally:
                self.finish_turn()
                with self._state_lock:
                    self._background_threads.discard(thread)

        thread = threading.Thread(target=run, daemon=True)
        with self._state_lock:
            self._background_threads.add(thread)
            thread.start()
        return True

    def try_begin_turn(self) -> bool:
        with self._state_lock:
            if self._busy or self._stop.is_set():
                return False
            self._busy = True
            return True

    def _begin_worker_turn(self) -> bool:
        # worker 可能与记忆反馈等受管 background turn 竞争；必须串行取得同一个 busy 所有权。
        while not self._stop.is_set():
            with self._state_lock:
                if not self._busy:
                    self._busy = True
                    return True
            time.sleep(0.005)
        return False

    def finish_turn(self) -> None:
        with self._state_lock:
            self._busy = False

    def is_busy(self) -> bool:
        with self._state_lock:
            return self._busy

    def is_worker_thread(self) -> bool:
        return self._worker is not None and threading.current_thread() is self._worker

    def raise_if_stopping(self) -> None:
        if self._stop.is_set():
            raise AgentExecutionCancelled("agent lifecycle deactivated")

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

            if not self._begin_worker_turn():
                self._control.command_queue.task_done()
                break
            try:
                self._publish_execution(self._control.execution_started(item))
                self._before_execute(item)
                self._execute_item(item)
                self._publish_execution(
                    self._control.execution_finished(
                        item, success=True, reason="completed"
                    )
                )
            except AgentExecutionCancelled as exc:
                self._publish_execution(
                    self._control.execution_finished(
                        item, success=False, reason=str(exc)
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

    def stop(self, timeout_s: float = 5.0) -> bool:
        """请求所有 turn 协作退出并等待；返回 False 表示仍有资源未静默。"""

        self._stop.set()
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        with self._state_lock:
            threads = tuple(self._background_threads)
            if self._worker is not None:
                threads += (self._worker,)
        for thread in threads:
            if thread is threading.current_thread():
                continue
            thread.join(timeout=max(0.0, deadline - time.monotonic()))
        with self._state_lock:
            self._discard_finished_threads_locked()
            if self._worker is not None and not self._worker.is_alive():
                self._worker = None
            return not any(thread.is_alive() for thread in threads)

    def _discard_finished_threads_locked(self) -> None:
        self._background_threads = {
            thread for thread in self._background_threads if thread.is_alive()
        }
