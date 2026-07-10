"""轻量中文机器人命令 NLU。

这个模块用于连续语音场景：一条 ASR final 里可能包含多个控制意图，例如
“向右转，向前走一秒”。它不是大模型，而是一个可解释、可测试的本地小模型：

1. 先用动作锚点构造候选片段，支持无标点的“右转向前走一秒”。
2. 再用字符 n-gram 原型模型给片段打分，判断是否像机器人控制命令。
3. 最后用确定性槽位抽取生成 ActionCommand，仍交给 ActionGuard 做安全校验。

这样比纯字符串 split 更稳，也避免引入 torch/transformers 等重依赖。
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, List

from .navigation_phrases import (
    DEFAULT_PATROL_WAYPOINTS,
    extract_waypoints,
    is_multi_stop_route_request,
    is_navigation_cancel,
    is_patrol_request,
    is_waypoint_sequence_request,
    resolve_place,
)
from .types import ActionCommand


_CHINESE_NUMBERS = {"一": 1, "二": 2, "两": 2, "三": 3, "四": 4, "五": 5}
_CHINESE_DIGITS = {
    "零": 0,
    "〇": 0,
    "一": 1,
    "二": 2,
    "两": 2,
    "三": 3,
    "四": 4,
    "五": 5,
    "六": 6,
    "七": 7,
    "八": 8,
    "九": 9,
}
_NUMBER_TOKEN = r"(?:\d+(?:\.\d+)?|[零〇一二两三四五六七八九十百点]+|半)"
_PUNCTUATION = re.compile(r"[，。！？!?\s、,.；;：:]")
_CONNECTORS = ("然后", "接着", "随后", "再", "并且")
_QUESTION_OR_NEGATION = (
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
    "不要",
    "禁止",
)
_UNSAFE_COMBINATION_MARKERS = ("一边", "同时", "高速")
_UNSAFE_ROTATION_MARKERS = ("旋转", "转")
_UNSAFE_SPEED_MARKERS = ("高速", "全速", "最快", "最大速度", "冲过去")
_RETRY_PROMPTS = {
    "missing_led_color": "听到了灯光命令，但颜色不完整，请重说，例如：把灯设成蓝色",
    "missing_turn_direction": "听到了转向角度，但方向不完整，请重说：左转或右转九十度",
}


def _clean(text: str) -> str:
    cleaned = _PUNCTUATION.sub("", text.lower())
    for connector in _CONNECTORS:
        cleaned = cleaned.replace(connector, "")
    return cleaned


def _ngrams(text: str, n: int = 2) -> set[str]:
    if not text:
        return set()
    if len(text) <= n:
        return {text}
    return {text[index : index + n] for index in range(len(text) - n + 1)}


def _parse_number(token: str) -> float | None:
    token = token.strip()
    if token == "半":
        return 0.5
    try:
        return float(token)
    except ValueError:
        pass
    if "点" in token:
        integer_text, decimal_text = token.split("点", 1)
        integer = _parse_number(integer_text or "零")
        if integer is None or not decimal_text:
            return None
        decimal_digits = []
        for char in decimal_text:
            if char not in _CHINESE_DIGITS:
                return None
            decimal_digits.append(str(_CHINESE_DIGITS[char]))
        return float(f"{int(integer)}.{''.join(decimal_digits)}")
    if not token or any(
        char not in _CHINESE_DIGITS and char not in ("十", "百") for char in token
    ):
        return None
    total = 0
    current = 0
    for char in token:
        if char in _CHINESE_DIGITS:
            current = _CHINESE_DIGITS[char]
        elif char == "十":
            total += (current or 1) * 10
            current = 0
        elif char == "百":
            total += (current or 1) * 100
            current = 0
    return float(total + current)


def _quantity_match(pattern: str, text: str) -> tuple[float | None, tuple[int, int] | None]:
    match = re.search(pattern, text, re.IGNORECASE)
    if not match:
        return None, None
    return _parse_number(match.group("value")), match.span()


def _duration(text: str, default: float = 1.0) -> float:
    if "半秒" in text:
        return 0.5
    match = re.search(rf"(?P<value>{_NUMBER_TOKEN})\s*(?:秒|s)", text, re.IGNORECASE)
    if match:
        value = _parse_number(match.group("value"))
        if value is not None:
            return min(10.0, max(0.1, value))
    return default


def _speed(
    text: str, default: float = 0.2
) -> tuple[float, tuple[int, int] | None, bool]:
    patterns = (
        rf"每秒\s*(?P<value>{_NUMBER_TOKEN})\s*(?:米|m)",
        rf"(?P<value>{_NUMBER_TOKEN})\s*(?:米|m)\s*(?:每秒|/s)",
    )
    for pattern in patterns:
        value, span = _quantity_match(pattern, text)
        if value is not None:
            return min(0.5, max(0.05, value)), span, True
    if "慢速" in text or "慢一点" in text:
        return 0.1, None, True
    if "快速" in text or "快一点" in text:
        return 0.3, None, True
    return default, None, False


def _distance(text: str, speed_span: tuple[int, int] | None) -> float | None:
    for match in re.finditer(
        rf"(?P<value>{_NUMBER_TOKEN})\s*(?:米|m)(?!\s*(?:每秒|/s))",
        text,
        re.IGNORECASE,
    ):
        if speed_span and match.start() < speed_span[1] and match.end() > speed_span[0]:
            continue
        value = _parse_number(match.group("value"))
        if value is not None:
            return min(5.0, max(0.05, value))
    return None


def _angle(text: str) -> float | None:
    value, _ = _quantity_match(
        rf"(?P<value>{_NUMBER_TOKEN})\s*(?:度|°)", text
    )
    if value is None:
        return None
    return min(360.0, max(1.0, value))


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


_DEFAULT_PROTOTYPES = {
    "move_forward": ["向前走一秒", "前进一秒", "往前走", "向前", "前进"],
    "move_backward": ["后退一秒", "向后退", "往后走", "后退", "向后"],
    "turn_left": ["左转九十度", "向左转", "左转", "左转90度"],
    "turn_right": ["右转九十度", "向右转", "右转", "右转90度"],
    "stop": ["停下", "停止", "急停", "刹车", "别动"],
    "arc": ["绕圈", "画圆", "转圈"],
    "spin": ["原地转一圈", "旋转一圈", "转一圈"],
    "square": ["走正方形", "正方形", "方形巡游"],
    "demo": ["演示一下", "做个演示", "展示一下"],
    "wave": ["挥手", "挥手三次"],
    "set_led": ["开蓝灯", "把灯设为蓝色", "关灯"],
    "set_mode": ["开启自动避障", "开始沿墙行走", "退出自动模式"],
    "navigate_to": ["去门口", "去客厅", "前往书桌", "回到起点", "去封闭区"],
    "follow_waypoints": ["开始巡航", "依次去门口书桌起点", "多点巡航"],
    "cancel_navigation": ["取消导航", "停止巡航"],
}


@dataclass(frozen=True)
class ParsedCommand:
    intent: str
    span_text: str
    actions: List[ActionCommand]
    confidence: float
    slots: dict[str, float | str | int] = field(default_factory=dict)


@dataclass(frozen=True)
class NluResult:
    source_text: str
    commands: List[ParsedCommand] = field(default_factory=list)
    reason: str = ""

    @property
    def accepted(self) -> bool:
        return bool(self.commands)

    @property
    def retry_prompt(self) -> str:
        """仅对确定缺槽位的控制语句要求重说，普通聊天仍可回退到 LLM。"""
        return _RETRY_PROMPTS.get(self.reason, "")


class CharacterNgramIntentModel:
    """字符 n-gram 原型分类器。

    训练好的“模型”本质是一组意图原型短句。运行时把输入片段和每个意图的
    n-gram 集合做余弦相似度，足够覆盖本项目固定动作域，且没有额外依赖。
    """

    def __init__(self, prototypes: dict[str, list[str]] | None = None):
        self._prototypes = prototypes or _DEFAULT_PROTOTYPES
        self._vectors = {
            intent: self._vectorize(samples)
            for intent, samples in self._prototypes.items()
        }

    @classmethod
    def from_json(cls, path: Path | str) -> "CharacterNgramIntentModel":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls(data.get("intents", _DEFAULT_PROTOTYPES))

    def predict(self, text: str, candidates: Iterable[str] | None = None) -> tuple[str, float]:
        vector = self._vectorize([text])
        candidate_names = list(candidates or self._vectors.keys())
        scored = [
            (intent, self._cosine(vector, self._vectors.get(intent, {})))
            for intent in candidate_names
        ]
        return max(scored, key=lambda item: item[1]) if scored else ("", 0.0)

    @staticmethod
    def _vectorize(samples: Iterable[str]) -> dict[str, float]:
        counts: dict[str, float] = {}
        for sample in samples:
            for token in _ngrams(_clean(sample)):
                counts[token] = counts.get(token, 0.0) + 1.0
        return counts

    @staticmethod
    def _cosine(left: dict[str, float], right: dict[str, float]) -> float:
        if not left or not right:
            return 0.0
        dot = sum(value * right.get(key, 0.0) for key, value in left.items())
        left_norm = math.sqrt(sum(value * value for value in left.values()))
        right_norm = math.sqrt(sum(value * value for value in right.values()))
        return dot / (left_norm * right_norm) if left_norm and right_norm else 0.0


class CommandNLU:
    """把一句 ASR final 解析为一个或多个机器人动作。"""

    _ANCHORS = (
        ("stop", ("停下", "停止", "急停", "刹车", "别动")),
        ("square", ("走正方形", "正方形", "方形巡游")),
        ("demo", ("演示一下", "做个演示", "展示一下")),
        ("spin", ("原地转一圈", "旋转一圈", "转一圈")),
        ("arc", ("绕圈", "画圆", "转圈")),
        ("move_forward", ("向前走", "往前走", "向前", "前进")),
        ("move_backward", ("向后退", "往后退", "向后", "后退")),
        ("turn_left", ("向左转", "左转")),
        ("turn_right", ("向右转", "右转")),
        ("wave", ("挥手",)),
        ("set_led", ("灯",)),
        ("set_mode", ("自动避障", "避障模式", "沿墙", "贴墙", "手动模式", "手动控制", "退出自动")),
        ("cancel_navigation", ("取消导航", "停止导航", "退出导航", "取消巡航", "停止巡航")),
        (
            "follow_waypoints",
            ("开始巡航", "巡航一圈", "巡逻一圈", "开始巡逻", "多点巡航", "巡逻", "巡航", "依次", "按顺序"),
        ),
        ("navigate_to", ("导航到", "前往", "回到", "返回", "去", "到")),
    )

    def __init__(
        self,
        *,
        enabled: bool = True,
        min_confidence: float = 0.18,
        model: CharacterNgramIntentModel | None = None,
    ):
        self.enabled = enabled
        self.min_confidence = min_confidence
        self.model = model or CharacterNgramIntentModel()

    def parse(self, text: str) -> NluResult:
        source = text.strip()
        normalized = _clean(source)
        if not self.enabled or not normalized:
            return NluResult(source, reason="disabled_or_empty")
        if any(marker in normalized for marker in _QUESTION_OR_NEGATION):
            return NluResult(source, reason="blocked_semantic")
        if any(marker in normalized for marker in _UNSAFE_SPEED_MARKERS):
            return NluResult(source, reason="blocked_unsafe_speed")
        if any(marker in normalized for marker in _UNSAFE_COMBINATION_MARKERS) and any(
            marker in normalized for marker in _UNSAFE_ROTATION_MARKERS
        ):
            # 连续语音队列会优先走 NLU，如果这里不拦截，“一边前进一边高速旋转”
            # 可能被锚点模型截成一个普通 move。安全否定必须早于动作锚点抽取。
            return NluResult(source, reason="blocked_unsafe_combination")

        anchors = self._find_anchors(normalized)
        if not anchors:
            # 真人录音里“左转九十度”曾被提交为“我九十”。方向无法安全猜测，
            # 与其让 LLM 产生不确定动作，不如明确要求用户重复这一条命令。
            if self._looks_like_turn_angle_without_direction(normalized):
                return NluResult(source, reason="missing_turn_direction")
            return NluResult(source, reason="no_anchor")

        # 停止类命令是安全优先：一句话里只要出现急停，就不再排后续普通动作。
        if any(intent == "stop" for _, intent in anchors):
            return NluResult(
                source,
                [ParsedCommand("stop", "停下", [ActionCommand("stop", {})], 1.0)],
            )
        if any(intent == "cancel_navigation" for _, intent in anchors) or is_navigation_cancel(normalized):
            return NluResult(
                source,
                [
                    ParsedCommand(
                        "cancel_navigation",
                        "取消导航",
                        [ActionCommand("cancel_navigation", {})],
                        1.0,
                    )
                ],
            )
        whole_waypoints = extract_waypoints(normalized)
        if (
            is_waypoint_sequence_request(source) and len(whole_waypoints) >= 2
        ) or (is_multi_stop_route_request(source) and len(whole_waypoints) >= 3):
            return NluResult(
                source,
                [
                    ParsedCommand(
                        "follow_waypoints",
                        normalized,
                        [
                            ActionCommand(
                                "follow_waypoints",
                                {
                                    "waypoints": whole_waypoints,
                                    "number_of_loops": 1,
                                },
                            )
                        ],
                        1.0,
                    )
                ],
            )
        if is_patrol_request(normalized):
            return NluResult(
                source,
                [
                    ParsedCommand(
                        "follow_waypoints",
                        normalized,
                        [
                            ActionCommand(
                                "follow_waypoints",
                                {
                                    "waypoints": whole_waypoints or DEFAULT_PATROL_WAYPOINTS,
                                    "number_of_loops": 1,
                                },
                            )
                        ],
                        1.0,
                    )
                ],
            )

        commands: list[ParsedCommand] = []
        for index, (start, anchor_intent) in enumerate(anchors):
            end = anchors[index + 1][0] if index + 1 < len(anchors) else len(normalized)
            # “以每秒 0.2 米向前走一米”的速度修饰语位于首个动作锚点之前；
            # 第一个片段保留前缀，避免分类成功后槽位却被切掉。
            segment = normalized[0 if index == 0 else start:end]
            intent, confidence = self.model.predict(segment, [anchor_intent])
            if confidence < self.min_confidence:
                continue
            actions, slots = self._actions_and_slots_for(intent, segment)
            if actions:
                commands.append(ParsedCommand(intent, segment, actions, confidence, slots))
        if commands:
            return NluResult(source, commands)
        if any(intent == "set_led" for _, intent in anchors):
            return NluResult(source, reason="missing_led_color")
        return NluResult(source, reason="low_confidence")

    @staticmethod
    def _looks_like_turn_angle_without_direction(normalized: str) -> bool:
        if any(marker in normalized for marker in ("左", "右", "转")):
            return False
        angle = r"(?:四十五|九十|一百八十|三百六十|45|90|180|360)"
        return re.fullmatch(rf"(?:我|向|往)?{angle}(?:度)?", normalized) is not None

    def _find_anchors(self, normalized: str) -> list[tuple[int, str]]:
        matches: list[tuple[int, int, str]] = []
        for intent, words in self._ANCHORS:
            for word in words:
                start = normalized.find(word)
                while start >= 0:
                    matches.append((start, len(word), intent))
                    start = normalized.find(word, start + 1)
        matches.sort(key=lambda item: (item[0], -item[1]))
        result: list[tuple[int, str]] = []
        occupied_until = -1
        for start, length, intent in matches:
            if start < occupied_until:
                continue
            result.append((start, intent))
            occupied_until = start + length
        return result

    def _actions_and_slots_for(
        self, intent: str, segment: str
    ) -> tuple[list[ActionCommand], dict[str, float | str | int]]:
        if intent == "move_forward":
            return self._motion_action_and_slots(segment, "forward")
        if intent == "move_backward":
            return self._motion_action_and_slots(segment, "backward")
        if intent == "turn_left":
            return self._turn_action_and_slots(segment, "left")
        if intent == "turn_right":
            return self._turn_action_and_slots(segment, "right")
        if intent == "spin":
            return [ActionCommand("turn", {"angular_z": 0.8, "duration_s": 7.85})], {
                "direction": "left",
                "angle_deg": 360.0,
                "angular_speed_rps": 0.8,
                "duration_s": 7.85,
            }
        if intent == "arc":
            return [ActionCommand("arc", {"linear_x": 0.12, "angular_z": 0.45, "duration_s": 6.0})], {}
        if intent == "square":
            return _square_sequence(), {}
        if intent == "demo":
            return _demo_sequence(), {}
        if intent == "wave":
            count = _count(segment)
            return [ActionCommand("wave", {"count": count})], {"count": float(count)}
        if intent == "set_led":
            colors = {"红": "red", "绿": "green", "蓝": "blue", "黄": "yellow", "白": "white", "关闭": "off", "关": "off"}
            for keyword, color in colors.items():
                if keyword in segment:
                    return [ActionCommand("set_led", {"color": color})], {"color": color}
        if intent == "set_mode":
            if "沿墙" in segment or "贴墙" in segment:
                return [ActionCommand("set_mode", {"mode": "wall_following"})], {"mode": "wall_following"}
            if "自动避障" in segment or "避障模式" in segment:
                return [ActionCommand("set_mode", {"mode": "obstacle_avoidance"})], {"mode": "obstacle_avoidance"}
            if "手动" in segment or "退出自动" in segment:
                return [ActionCommand("set_mode", {"mode": "manual"})], {"mode": "manual"}
        if intent == "navigate_to":
            target = resolve_place(segment)
            if target:
                return [ActionCommand("navigate_to", {"target": target})], {"place": target}
        if intent == "follow_waypoints":
            waypoints = extract_waypoints(segment) or (
                DEFAULT_PATROL_WAYPOINTS if is_patrol_request(segment) else []
            )
            if len(waypoints) >= 2:
                return [
                    ActionCommand(
                        "follow_waypoints",
                        {"waypoints": waypoints, "number_of_loops": 1},
                    )
                ], {}
        return [], {}

    @staticmethod
    def _motion_action_and_slots(
        segment: str, direction: str
    ) -> tuple[list[ActionCommand], dict[str, float | str | int]]:
        speed, speed_span, explicit_speed = _speed(segment)
        distance = _distance(segment, speed_span)
        if distance is not None and distance / speed > 10.0 and not explicit_speed:
            # 未指定速度时可在 ActionGuard 的 0.5m/s 上限内提速，尽量保持单动作。
            speed = min(0.5, distance / 10.0)
        total_duration = distance / speed if distance is not None else _duration(segment)
        segment_count = max(1, math.ceil(total_duration / 10.0))
        segment_duration = round(total_duration / segment_count, 3)
        signed_speed = speed if direction == "forward" else -speed
        slots: dict[str, float | str | int] = {
            "direction": direction,
            "speed_mps": speed,
            "duration_s": round(total_duration, 3),
        }
        if distance is not None:
            # 保持学习笔记可解释的固定顺序：方向、距离、速度、换算后的时长。
            slots = {
                "direction": direction,
                "distance_m": distance,
                "speed_mps": speed,
                "duration_s": round(total_duration, 3),
            }
            if segment_count > 1:
                # 单个 RobotCommand 最长 10 秒；显式慢速长距离拆成顺序 primitive，
                # 不提速、不截断，也不让 fallback 把它误解为默认一秒。
                slots["segment_count"] = segment_count
        actions = [
            ActionCommand(
                "move", {"linear_x": signed_speed, "duration_s": segment_duration}
            )
            for _ in range(segment_count)
        ]
        return actions, slots

    @staticmethod
    def _turn_action_and_slots(
        segment: str, direction: str
    ) -> tuple[list[ActionCommand], dict[str, float | str | int]]:
        explicit_angle = _angle(segment)
        angle = explicit_angle if explicit_angle is not None else 90.0
        angular_speed = 0.8 if math.radians(angle) / 0.6 > 10.0 else 0.6
        if explicit_angle is None and "秒" in segment:
            duration = _duration(segment)
            angle = round(math.degrees(angular_speed * duration), 3)
        elif angle == 90.0:
            # 保持历史动作参数兼容，90° 使用项目原有标定值 2.6 秒。
            duration = 2.6
        else:
            duration = round(math.radians(angle) / angular_speed, 3)
        signed_speed = angular_speed if direction == "left" else -angular_speed
        slots: dict[str, float | str | int] = {
            "direction": direction,
            "angle_deg": angle,
            "angular_speed_rps": angular_speed,
            "duration_s": duration,
        }
        return [ActionCommand("turn", {"angular_z": signed_speed, "duration_s": duration})], slots
