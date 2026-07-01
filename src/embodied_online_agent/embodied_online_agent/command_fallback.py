import re
from typing import Optional

from .types import ActionCommand

_CHINESE_NUMBERS = {"一": 1, "二": 2, "两": 2, "三": 3, "四": 4, "五": 5}


def _duration(text: str, default: float = 1.0) -> float:
    if "半秒" in text:
        return 0.5
    match = re.search(r"(\d+(?:\.\d+)?)\s*(?:秒|s)", text, re.IGNORECASE)
    if match:
        return min(10.0, max(0.1, float(match.group(1))))
    match = re.search(r"([一二两三四五])秒", text)
    if match:
        return float(_CHINESE_NUMBERS[match.group(1)])
    return default


def _count(text: str) -> int:
    match = re.search(r"([1-5])\s*次", text)
    if match:
        return int(match.group(1))
    match = re.search(r"([一二两三四五])次", text)
    return _CHINESE_NUMBERS[match.group(1)] if match else 1


def parse_fallback_action(text: str) -> Optional[ActionCommand]:
    """Parse explicit commands for deterministic arbitration over model actions."""
    normalized = re.sub(r"[，。！？!?\s]", "", text.lower())
    if any(word in normalized for word in ("退出自动", "关闭自动", "停止自动")):
        return ActionCommand("set_mode", {"mode": "manual"})
    if "手动模式" in normalized or "手动控制" in normalized:
        return ActionCommand("set_mode", {"mode": "manual"})
    if "自动避障" in normalized or "避障模式" in normalized:
        return ActionCommand("set_mode", {"mode": "obstacle_avoidance"})
    if "沿墙" in normalized or "贴墙" in normalized:
        return ActionCommand("set_mode", {"mode": "wall_following"})
    if any(word in normalized for word in ("别动", "不要动")):
        return ActionCommand("stop", {})
    if should_block_model_actions(normalized):
        return None
    if any(word in normalized for word in ("停止", "停下", "急停")):
        return ActionCommand("stop", {})
    if "向前" in normalized or "前进" in normalized:
        return ActionCommand(
            "move", {"linear_x": 0.2, "duration_s": _duration(normalized)}
        )
    if "后退" in normalized or "向后" in normalized:
        return ActionCommand(
            "move", {"linear_x": -0.2, "duration_s": _duration(normalized)}
        )
    if "左转" in normalized or "向左转" in normalized:
        duration = (
            2.6
            if "九十度" in normalized or "90度" in normalized
            else _duration(normalized)
        )
        return ActionCommand(
            "turn", {"angular_z": 0.6, "duration_s": duration}
        )
    if "右转" in normalized or "向右转" in normalized:
        duration = (
            2.6
            if "九十度" in normalized or "90度" in normalized
            else _duration(normalized)
        )
        return ActionCommand(
            "turn", {"angular_z": -0.6, "duration_s": duration}
        )
    if re.search(r"挥.*手", normalized):
        return ActionCommand("wave", {"count": _count(normalized)})
    if "灯" in normalized:
        colors = {
            "红": "red",
            "绿": "green",
            "蓝": "blue",
            "黄": "yellow",
            "白": "white",
            "关闭": "off",
            "关": "off",
        }
        for keyword, color in colors.items():
            if keyword in normalized:
                return ActionCommand("set_led", {"color": color})
    return None


def should_block_model_actions(text: str) -> bool:
    normalized = re.sub(r"[，。！？!?\s]", "", text.lower())
    question_markers = (
        "什么",
        "为什么",
        "如何",
        "觉得",
        "意思",
        "怎么样",
        "多少",
        "能不能",
        "可不可以",
        "吗",
    )
    if any(word in normalized for word in question_markers):
        return True
    if any(word in normalized for word in ("不要", "别", "禁止")):
        return True
    return ("一边" in normalized or "同时" in normalized or "高速" in normalized) and (
        "旋转" in normalized or "转" in normalized
    )
