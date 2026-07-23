"""语音导航地点词表。

这里先用固定语义地点支撑 Gazebo/Nav2 demo。这样 Agent 侧只处理“去哪里”，
具体坐标可以由后续 Nav2 bridge 的 places.yaml 维护，避免把坐标写进 LLM 输出。
"""

from __future__ import annotations

import re


PLACE_ALIASES: dict[str, tuple[str, ...]] = {
    "home": ("起点", "原点", "回家", "回到起点", "返回起点", "回起点", "出发点", "home"),
    "door": ("门口", "门边", "门", "door"),
    "desk": ("桌子", "书桌", "桌子旁", "办公桌", "desk"),
    "living_room": ("客厅", "大厅", "livingroom"),
    "kitchen": ("厨房", "kitchen"),
    "entrance": ("入口", "玄关", "entrance"),
    "office": ("办公室", "办公区", "office"),
    "meeting_room": ("会议区", "会议室", "meetingroom"),
    "hallway": ("走廊", "过道", "hallway"),
    "charging_station": ("充电区", "充电桩", "充电站", "charger"),
    # 仅用于 Nav2 失败恢复验收：坐标故意配置在演示地图外，不能用于普通演示路线。
    "unreachable_zone": ("封闭区", "不可达区", "禁区", "unreachable"),
    "waypoint_a": ("a点", "A点", "一号点", "1号点"),
    "waypoint_b": ("b点", "B点", "二号点", "2号点"),
    "waypoint_c": ("c点", "C点", "三号点", "3号点"),
}

DEFAULT_PATROL_WAYPOINTS = ["door", "desk", "home"]


def normalize_place_text(text: str) -> str:
    return re.sub(r"[，。！？!?\s、,.；;：:]", "", text)


def resolve_place(text: str) -> str | None:
    normalized = normalize_place_text(text).lower()
    for place, aliases in PLACE_ALIASES.items():
        for alias in aliases:
            if alias.lower() in normalized:
                return place
    return None


def extract_waypoints(text: str) -> list[str]:
    normalized = normalize_place_text(text).lower()
    matches: list[tuple[int, str]] = []
    for place, aliases in PLACE_ALIASES.items():
        for alias in aliases:
            index = normalized.find(alias.lower())
            if index >= 0:
                matches.append((index, place))
                break
    matches.sort(key=lambda item: item[0])
    waypoints: list[str] = []
    for _, place in matches:
        if not waypoints or waypoints[-1] != place:
            waypoints.append(place)
    return waypoints


def is_navigation_cancel(text: str) -> bool:
    normalized = normalize_place_text(text)
    return any(word in normalized for word in ("取消导航", "停止导航", "退出导航", "取消巡航", "停止巡航"))


def is_patrol_request(text: str) -> bool:
    normalized = normalize_place_text(text)
    return any(
        word in normalized
        for word in (
            "开始巡航",
            "巡航一圈",
            "巡逻一圈",
            "开始巡逻",
            "多点巡航",
            "巡逻",
            "巡航",
        )
    )


def is_waypoint_sequence_request(text: str) -> bool:
    normalized = normalize_place_text(text)
    return any(
        word in normalized
        for word in (
            "依次",
            "按顺序",
            "逐个",
            "逐一",
        )
    ) or is_patrol_request(normalized)


def is_multi_stop_route_request(text: str) -> bool:
    normalized = normalize_place_text(text)
    return "最后" in normalized and any(
        word in normalized for word in ("先", "再", "然后", "接着", "随后")
    )


def is_navigation_request(text: str) -> bool:
    normalized = normalize_place_text(text)
    return any(word in normalized for word in ("去", "到", "前往", "导航到", "回到", "返回"))
