"""frontier 探索完成策略；不依赖 ROS 消息和 Node。"""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Callable, Protocol

from .mapping_evidence import MappingEvidenceSnapshot, MappingEvidenceTracker
from .mission_executor import AutomaticMissionCancelled, CommandRequest
from .showcase_session import exploration_completion_reason


@dataclass(frozen=True, slots=True)
class FrontierMonitorConfig:
    timeout_s: float
    min_runtime_s: float
    stable_map_s: float
    completion_status: str
    min_known_cells: int
    min_occupied_cells: int
    min_mapping_path_m: float
    status_available: bool = True
    dry_run: bool = False
    dry_run_delay_s: float = 0.0


class ExplorerHealthPort(Protocol):
    def explorer_exited_unexpectedly(self) -> tuple[bool, int | None]: ...


class FrontierExplorationMonitor:
    """融合地图、里程和 explorer 健康状态，给出可审计结束原因。"""

    def __init__(
        self,
        evidence: MappingEvidenceTracker,
        explorer: ExplorerHealthPort,
        config: FrontierMonitorConfig,
        *,
        cancel_motion: Callable[[], None],
        log_info: Callable[[str], None] = lambda _message: None,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._evidence = evidence
        self._explorer = explorer
        self._config = config
        self._cancel_motion = cancel_motion
        self._log_info = log_info
        self._clock = clock
        self._sleep = sleep

    def wait(self, request: CommandRequest) -> str:
        if self._config.dry_run:
            return self._wait_dry_run(request)
        if not self._config.status_available:
            raise RuntimeError(
                "explore_lite_msgs is unavailable; run "
                "bash scripts/setup_frontier_exploration.sh"
            )

        started_at = self._clock()
        deadline = started_at + self._config.timeout_s
        with self._evidence.condition:
            while self._clock() < deadline:
                self._raise_if_canceled(request)
                elapsed = self._clock() - started_at
                snapshot = self._evidence.snapshot()
                self._reject_premature_explorer_completion(snapshot, elapsed)
                reason = self._completion_reason(snapshot, elapsed)
                if reason is not None:
                    return self._complete(snapshot, reason)
                exited, code = self._explorer.explorer_exited_unexpectedly()
                if exited:
                    raise RuntimeError(
                        f"frontier explorer exited unexpectedly code={code}"
                    )
                self._evidence.condition.wait(
                    timeout=min(0.5, max(0.0, deadline - self._clock()))
                )

        snapshot = self._evidence.snapshot()
        reason = self._completion_reason(
            snapshot,
            self._clock() - started_at,
            time_budget_reached=True,
        )
        if reason is not None:
            return self._complete(snapshot, reason)
        stats = snapshot.map_stats or {}
        raise TimeoutError(
            "frontier exploration did not complete before timeout "
            f"known={stats.get('known_cells', 0)} "
            f"occupied={stats.get('occupied_cells', 0)} "
            f"path={snapshot.mapping_path_m:.2f}m"
        )

    def _wait_dry_run(self, request: CommandRequest) -> str:
        deadline = self._clock() + self._config.dry_run_delay_s
        while self._clock() < deadline:
            self._raise_if_canceled(request)
            self._sleep(0.05)
        return "dry_run"

    def _raise_if_canceled(self, request: CommandRequest) -> None:
        if not request.canceled:
            return
        # 急停必须先旁路普通队列发布 stop，再向事务层传播取消原因。
        self._cancel_motion()
        raise AutomaticMissionCancelled("automatic mission canceled")

    def _reject_premature_explorer_completion(
        self,
        snapshot: MappingEvidenceSnapshot,
        elapsed_s: float,
    ) -> None:
        if (
            snapshot.explore_status != self._config.completion_status
            or elapsed_s < self._config.min_runtime_s
        ):
            return
        stats = snapshot.map_stats or {}
        if stats.get("known_cells", 0) < self._config.min_known_cells:
            raise RuntimeError(
                "frontier exploration ended before known-cell threshold"
            )
        if stats.get("occupied_cells", 0) < self._config.min_occupied_cells:
            raise RuntimeError(
                "frontier exploration ended before occupied-cell threshold"
            )
        if snapshot.mapping_path_m < self._config.min_mapping_path_m:
            raise RuntimeError(
                "frontier exploration ended before mapping-path threshold "
                f"({snapshot.mapping_path_m:.2f}m < "
                f"{self._config.min_mapping_path_m:.2f}m)"
            )

    def _completion_reason(
        self,
        snapshot: MappingEvidenceSnapshot,
        elapsed_s: float,
        *,
        time_budget_reached: bool = False,
    ) -> str | None:
        stats = snapshot.map_stats or {}
        return exploration_completion_reason(
            status=snapshot.explore_status,
            completion_status=self._config.completion_status,
            elapsed_s=elapsed_s,
            min_runtime_s=self._config.min_runtime_s,
            known_cells=stats.get("known_cells", 0),
            occupied_cells=stats.get("occupied_cells", 0),
            min_known_cells=self._config.min_known_cells,
            min_occupied_cells=self._config.min_occupied_cells,
            mapping_path_m=snapshot.mapping_path_m,
            min_mapping_path_m=self._config.min_mapping_path_m,
            seconds_since_map_growth=(
                self._clock() - snapshot.last_map_growth_at
            ),
            stable_map_s=self._config.stable_map_s,
            time_budget_reached=time_budget_reached,
        )

    def _complete(
        self,
        snapshot: MappingEvidenceSnapshot,
        reason: str,
    ) -> str:
        self._evidence.completion_reason = reason
        stats = snapshot.map_stats or {}
        self._log_info(
            f"frontier exploration complete reason={reason}: "
            f"known={stats.get('known_cells', 0)} "
            f"occupied={stats.get('occupied_cells', 0)} "
            f"path={snapshot.mapping_path_m:.2f}m"
        )
        return reason
