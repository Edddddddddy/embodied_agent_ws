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


_TRUNCATED_AUTOMATIC_MISSION_FINALS = frozenset({"开始自动"})


def normalize_session_text(text: str) -> str:
    return re.sub(r"[\s，。！？!?、,；;：:]", "", text).lower()


def is_truncated_automatic_mission_text(text: str) -> bool:
    """识别已知的短 final，但只接受完整相等，避免普通“开始……”被误触发。"""

    return normalize_session_text(text) in _TRUNCATED_AUTOMATIC_MISSION_FINALS


def is_automatic_mission_cancel_text(text: str) -> bool:
    """急停语义必须在高层任务忙碌时仍能旁路普通命令队列。"""

    normalized = normalize_session_text(text)
    return any(
        phrase in normalized
        for phrase in ("停下", "急停", "停止自动任务", "取消自动任务")
    )


def parse_mapping_bootstrap_route(config: dict) -> list[tuple[str, str, str]]:
    """读取自动脱离角落的短路线，并限制为可被 Guard 审计的运动原语。"""

    raw_route = config.get("bootstrap_route", [])
    if not isinstance(raw_route, list):
        raise ValueError("automatic_exploration.bootstrap_route must be a list")
    route: list[tuple[str, str, str]] = []
    for index, raw_step in enumerate(raw_route):
        if not isinstance(raw_step, dict):
            raise ValueError(f"bootstrap_route[{index}] must be a mapping")
        text = str(raw_step.get("text", "")).strip()
        action = str(raw_step.get("action", "")).strip()
        label = str(raw_step.get("label", text)).strip()
        if not text or not label:
            raise ValueError(f"bootstrap_route[{index}] requires label/text")
        if action not in {"move", "turn"}:
            # bootstrap 只负责从充电角落进入开放区；目标点导航仍必须等地图保存、
            # AMCL/Nav2 切换完成后执行，不能在未知地图阶段偷跑语义导航。
            raise ValueError(
                f"bootstrap_route[{index}].action must be move/turn"
            )
        route.append((label, text, action))
    return route


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
    mapping_path_m: float,
    min_mapping_path_m: float,
    seconds_since_map_growth: float,
    stable_map_s: float,
    time_budget_reached: bool = False,
) -> str | None:
    """融合探索器事件和地图覆盖平台期，返回可审计的结束原因。"""

    if elapsed_s < min_runtime_s:
        return None
    acceptance_ready = (
        known_cells >= min_known_cells
        and occupied_cells >= min_occupied_cells
        and mapping_path_m >= min_mapping_path_m
    )
    if status == completion_status and acceptance_ready:
        return "no_frontiers"
    if acceptance_ready and seconds_since_map_growth >= stable_map_s:
        return "coverage_plateau"
    # 探索时间预算不是“必须清空所有 frontier”。当可验收覆盖已经达成时，
    # 保存仍在增长的当前地图比无限追逐家具背后的边界更符合任务语义。
    if acceptance_ready and time_budget_reached:
        return "time_budget_coverage"
    return None


def parse_session_command(text: str) -> SessionCommand | None:
    """只识别高确定性的系统命令，普通聊天和机器人动作继续交给 Agent。"""

    normalized = normalize_session_text(text)
    if is_truncated_automatic_mission_text(normalized) or any(
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
