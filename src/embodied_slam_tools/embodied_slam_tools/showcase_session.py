"""真实感 SLAM Demo 的会话状态机与中文系统意图解析。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import re


class SessionCommand(IntEnum):
    SAVE_MAP = 1
    START_NAVIGATION = 2
    SAVE_AND_START_NAVIGATION = 3
    STOP_SESSION = 4
    RUN_AUTOMATIC_MISSION = 5


class SessionPhase(IntEnum):
    STOPPED = 0
    STARTING_MAPPING = 1
    MAPPING = 2
    SAVING_MAP = 3
    MAP_SAVED = 4
    SWITCHING_TO_NAVIGATION = 5
    STARTING_NAVIGATION = 6
    NAVIGATING = 7
    FAILED = 8
    STOPPING = 9
    AUTOMATIC_MAPPING = 10
    AUTOMATIC_NAVIGATING = 11
    MISSION_COMPLETED = 12


@dataclass(frozen=True)
class SessionSnapshot:
    phase: SessionPhase
    map_saved: bool = False
    map_yaml_path: str = ""
    detail: str = ""


def normalize_session_text(text: str) -> str:
    return re.sub(r"[\s，。！？!?、,；;：:]", "", text).lower()


def is_automatic_mission_cancel_text(text: str) -> bool:
    """急停语义必须在高层任务忙碌时仍能旁路普通命令队列。"""

    normalized = normalize_session_text(text)
    return any(
        phrase in normalized
        for phrase in ("停下", "急停", "停止自动任务", "取消自动任务")
    )


def exploration_completion_reason(
    *,
    status: str,
    completion_status: str,
    elapsed_s: float,
    min_runtime_s: float,
    known_cells: int,
    occupied_cells: int,
    min_known_cells: int,
    min_occupied_cells: int,
    seconds_since_map_growth: float,
    stable_map_s: float,
) -> str | None:
    """融合探索器事件和地图覆盖平台期，返回可审计的结束原因。"""

    if elapsed_s < min_runtime_s:
        return None
    coverage_ready = (
        known_cells >= min_known_cells and occupied_cells >= min_occupied_cells
    )
    if status == completion_status and coverage_ready:
        return "no_frontiers"
    if coverage_ready and seconds_since_map_growth >= stable_map_s:
        return "coverage_plateau"
    return None


def parse_session_command(text: str) -> SessionCommand | None:
    """只识别高确定性的系统命令，普通聊天和机器人动作继续交给 Agent。"""

    normalized = normalize_session_text(text)
    if any(
        phrase in normalized
        for phrase in (
            "开始自动巡检建图",
            "开始自动建图",
            "自动建图并导航",
            "自动巡检建图",
            "开始自主建图",
        )
    ):
        return SessionCommand.RUN_AUTOMATIC_MISSION
    if any(
        phrase in normalized
        for phrase in (
            "保存地图并开始导航",
            "保存地图然后开始导航",
            "保存并开始导航",
            "存图并开始导航",
        )
    ):
        return SessionCommand.SAVE_AND_START_NAVIGATION
    if any(
        phrase in normalized
        for phrase in ("开始导航", "进入导航模式", "切换到导航", "切换导航模式")
    ):
        return SessionCommand.START_NAVIGATION
    if any(phrase in normalized for phrase in ("保存地图", "保存当前地图", "存图")):
        return SessionCommand.SAVE_MAP
    if any(
        phrase in normalized
        for phrase in (
            "结束建图演示",
            "退出建图演示",
            "停止建图会话",
            "停止自动任务",
            "取消自动任务",
        )
    ):
        return SessionCommand.STOP_SESSION
    return None


class ShowcaseSessionStateMachine:
    """把进程操作与合法阶段转换分开，便于无 ROS 单测和失败恢复。"""

    def __init__(self) -> None:
        self._snapshot = SessionSnapshot(SessionPhase.STOPPED)

    @property
    def snapshot(self) -> SessionSnapshot:
        return self._snapshot

    def transition(
        self,
        phase: SessionPhase,
        *,
        detail: str,
        map_saved: bool | None = None,
        map_yaml_path: str | None = None,
    ) -> SessionSnapshot:
        previous = self._snapshot
        self._snapshot = SessionSnapshot(
            phase=phase,
            map_saved=previous.map_saved if map_saved is None else map_saved,
            map_yaml_path=(
                previous.map_yaml_path
                if map_yaml_path is None
                else map_yaml_path
            ),
            detail=detail,
        )
        return self._snapshot

    def validate(self, command: SessionCommand) -> tuple[bool, str]:
        phase = self._snapshot.phase
        if phase in {
            SessionPhase.STARTING_MAPPING,
            SessionPhase.SAVING_MAP,
            SessionPhase.SWITCHING_TO_NAVIGATION,
            SessionPhase.STARTING_NAVIGATION,
            SessionPhase.STOPPING,
            SessionPhase.AUTOMATIC_MAPPING,
            SessionPhase.AUTOMATIC_NAVIGATING,
        }:
            return False, f"session busy in phase={phase.name.lower()}"
        if (
            command == SessionCommand.RUN_AUTOMATIC_MISSION
            and phase != SessionPhase.MAPPING
        ):
            return False, "automatic mission can only start while mapping is ready"
        if command in {
            SessionCommand.SAVE_MAP,
            SessionCommand.SAVE_AND_START_NAVIGATION,
        } and phase not in {SessionPhase.MAPPING, SessionPhase.MAP_SAVED}:
            return False, "map can only be saved while mapping is active"
        if command == SessionCommand.START_NAVIGATION:
            if phase in {SessionPhase.NAVIGATING, SessionPhase.MISSION_COMPLETED}:
                return True, "navigation already active"
            if phase != SessionPhase.MAP_SAVED or not self._snapshot.map_saved:
                return False, "save the map before starting navigation"
        if command == SessionCommand.STOP_SESSION and phase == SessionPhase.STOPPED:
            return True, "session already stopped"
        return True, "accepted"
