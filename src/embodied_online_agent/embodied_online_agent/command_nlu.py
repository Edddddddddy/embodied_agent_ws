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

from .types import ActionCommand


_CHINESE_NUMBERS = {"一": 1, "二": 2, "两": 2, "三": 3, "四": 4, "五": 5}
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
}


@dataclass(frozen=True)
class ParsedCommand:
    intent: str
    span_text: str
    actions: List[ActionCommand]
    confidence: float


@dataclass(frozen=True)
class NluResult:
    source_text: str
    commands: List[ParsedCommand] = field(default_factory=list)
    reason: str = ""

    @property
    def accepted(self) -> bool:
        return bool(self.commands)


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

        anchors = self._find_anchors(normalized)
        if not anchors:
            return NluResult(source, reason="no_anchor")

        # 停止类命令是安全优先：一句话里只要出现急停，就不再排后续普通动作。
        if any(intent == "stop" for _, intent in anchors):
            return NluResult(
                source,
                [ParsedCommand("stop", "停下", [ActionCommand("stop", {})], 1.0)],
            )

        commands: list[ParsedCommand] = []
        for index, (start, anchor_intent) in enumerate(anchors):
            end = anchors[index + 1][0] if index + 1 < len(anchors) else len(normalized)
            segment = normalized[start:end]
            intent, confidence = self.model.predict(segment, [anchor_intent])
            if confidence < self.min_confidence:
                continue
            actions = self._actions_for(intent, segment)
            if actions:
                commands.append(ParsedCommand(intent, segment, actions, confidence))
        return NluResult(source, commands, "" if commands else "low_confidence")

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

    def _actions_for(self, intent: str, segment: str) -> list[ActionCommand]:
        if intent == "move_forward":
            return [ActionCommand("move", {"linear_x": 0.2, "duration_s": _duration(segment)})]
        if intent == "move_backward":
            return [ActionCommand("move", {"linear_x": -0.2, "duration_s": _duration(segment)})]
        if intent == "turn_left":
            duration = 2.6 if ("九十度" in segment or "90度" in segment or "秒" not in segment) else _duration(segment)
            return [ActionCommand("turn", {"angular_z": 0.6, "duration_s": duration})]
        if intent == "turn_right":
            duration = 2.6 if ("九十度" in segment or "90度" in segment or "秒" not in segment) else _duration(segment)
            return [ActionCommand("turn", {"angular_z": -0.6, "duration_s": duration})]
        if intent == "spin":
            return [ActionCommand("turn", {"angular_z": 0.8, "duration_s": 7.85})]
        if intent == "arc":
            return [ActionCommand("arc", {"linear_x": 0.12, "angular_z": 0.45, "duration_s": 6.0})]
        if intent == "square":
            return _square_sequence()
        if intent == "demo":
            return _demo_sequence()
        if intent == "wave":
            return [ActionCommand("wave", {"count": _count(segment)})]
        if intent == "set_led":
            colors = {"红": "red", "绿": "green", "蓝": "blue", "黄": "yellow", "白": "white", "关闭": "off", "关": "off"}
            for keyword, color in colors.items():
                if keyword in segment:
                    return [ActionCommand("set_led", {"color": color})]
        if intent == "set_mode":
            if "沿墙" in segment or "贴墙" in segment:
                return [ActionCommand("set_mode", {"mode": "wall_following"})]
            if "自动避障" in segment or "避障模式" in segment:
                return [ActionCommand("set_mode", {"mode": "obstacle_avoidance"})]
            if "手动" in segment or "退出自动" in segment:
                return [ActionCommand("set_mode", {"mode": "manual"})]
        return []
