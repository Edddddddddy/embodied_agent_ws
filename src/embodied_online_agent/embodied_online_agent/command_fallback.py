import re
from typing import List, Optional

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


def _square_sequence() -> List[ActionCommand]:
    actions: List[ActionCommand] = []
    for _ in range(4):
        actions.append(ActionCommand("move", {"linear_x": 0.18, "duration_s": 1.2}))
        actions.append(ActionCommand("turn", {"angular_z": 0.6, "duration_s": 2.6}))
    return actions


def _demo_sequence() -> List[ActionCommand]:
    return [
        ActionCommand("set_led", {"color": "blue"}),
        ActionCommand("wave", {"count": 2}),
        ActionCommand("move", {"linear_x": 0.18, "duration_s": 1.0}),
        ActionCommand("turn", {"angular_z": 0.8, "duration_s": 2.0}),
        ActionCommand("arc", {"linear_x": 0.12, "angular_z": 0.45, "duration_s": 6.0}),
        ActionCommand("stop", {}),
    ]


def parse_fallback_actions(text: str) -> List[ActionCommand]:
    """Parse explicit commands into deterministic primitive actions.

    组合动作在 Agent 层拆成 primitive command，后续发布器按 Action result 顺序推进。
    这样不会把复杂演示逻辑下沉到安全校验或仿真执行器里。
    """
    normalized = re.sub(r"[，。！？!?\s]", "", text.lower())
    if any(word in normalized for word in ("走正方形", "正方形", "方形巡游")):
        return _square_sequence()
    if any(word in normalized for word in ("演示一下", "做个演示", "展示一下")):
        return _demo_sequence()
    if any(word in normalized for word in ("退出自动", "关闭自动", "停止自动")):
        return [ActionCommand("set_mode", {"mode": "manual"})]
    if "手动模式" in normalized or "手动控制" in normalized:
        return [ActionCommand("set_mode", {"mode": "manual"})]
    if "自动避障" in normalized or "避障模式" in normalized:
        return [ActionCommand("set_mode", {"mode": "obstacle_avoidance"})]
    if "沿墙" in normalized or "贴墙" in normalized:
        return [ActionCommand("set_mode", {"mode": "wall_following"})]
    if any(word in normalized for word in ("别动", "不要动")):
        return [ActionCommand("stop", {})]
    if should_block_model_actions(normalized):
        return []
    if any(word in normalized for word in ("停止", "停下", "急停")):
        return [ActionCommand("stop", {})]
    if any(word in normalized for word in ("原地转一圈", "旋转一圈", "转一圈")):
        return [ActionCommand("turn", {"angular_z": 0.8, "duration_s": 7.85})]
    if any(word in normalized for word in ("绕圈", "画圆", "转圈")):
        return [
            ActionCommand(
                "arc", {"linear_x": 0.12, "angular_z": 0.45, "duration_s": 6.0}
            )
        ]
    if "向前" in normalized or "前进" in normalized:
        return [
            ActionCommand(
                "move", {"linear_x": 0.2, "duration_s": _duration(normalized)}
            )
        ]
    if "后退" in normalized or "向后" in normalized:
        return [
            ActionCommand(
                "move", {"linear_x": -0.2, "duration_s": _duration(normalized)}
            )
        ]
    if "左转" in normalized or "向左转" in normalized:
        duration = (
            2.6
            if "九十度" in normalized or "90度" in normalized
            else _duration(normalized)
        )
        return [ActionCommand("turn", {"angular_z": 0.6, "duration_s": duration})]
    if "右转" in normalized or "向右转" in normalized:
        duration = (
            2.6
            if "九十度" in normalized or "90度" in normalized
            else _duration(normalized)
        )
        return [ActionCommand("turn", {"angular_z": -0.6, "duration_s": duration})]
    if re.search(r"挥.*手", normalized):
        return [ActionCommand("wave", {"count": _count(normalized)})]
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
                return [ActionCommand("set_led", {"color": color})]
    return []


def parse_fallback_action(text: str) -> Optional[ActionCommand]:
    """Backward-compatible single-action facade used by older tests/callers."""
    actions = parse_fallback_actions(text)
    return actions[0] if actions else None


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
