"""ASR 后处理：把常见错词归一化为可执行的机器人命令文本。

真实麦克风场景里，ASR 往往不是完全失败，而是把短控制词识别成同音/近音字：
“前进”->“钱进”，“左转”->“作转”，“停下”->“亭下”。如果这些文本直接进入
会话状态机，尤其是停下/急停，就会错过抢占窗口。

这个模块是一个小而深的 seam：
只暴露 normalize(text)，内部可以使用 RapidFuzz、手写错词表或后续拼音相似度。
调用方不需要知道具体算法，只拿到规范化文本和可观测反馈。
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from difflib import SequenceMatcher
from pathlib import Path
from typing import Iterable, Protocol


_PUNCTUATION = re.compile(r"[，。！？!?\s]")


DEFAULT_ALIASES: tuple[tuple[str, str], ...] = (
    ("你好小志", "你好小智"),
    ("你好小治", "你好小智"),
    ("晓智", "小智"),
    ("晓志", "小智"),
    ("小志", "小智"),
    ("小治", "小智"),
    ("小只", "小智"),
    ("小之", "小智"),
    ("向钱", "向前"),
    ("相前", "向前"),
    ("像前", "向前"),
    ("项前", "向前"),
    ("前景", "前进"),
    ("前镜", "前进"),
    ("钱进", "前进"),
    ("后腿", "后退"),
    ("候退", "后退"),
    ("厚退", "后退"),
    ("作转", "左转"),
    ("坐转", "左转"),
    ("左传", "左转"),
    ("佐转", "左转"),
    ("有转", "右转"),
    ("又转", "右转"),
    ("右传", "右转"),
    ("右赚", "右转"),
    ("亭下", "停下"),
    ("听下", "停下"),
    ("停吓", "停下"),
    ("停夏", "停下"),
    ("吉停", "急停"),
    ("急亭", "急停"),
    ("让圈", "绕圈"),
    ("饶圈", "绕圈"),
    ("绕权", "绕圈"),
    ("花圆", "画圆"),
    ("画原", "画圆"),
    ("化圆", "画圆"),
    ("证方形", "正方形"),
    ("正方向", "正方形"),
    ("正方行", "正方形"),
    ("演示以下", "演示一下"),
    ("演示一夏", "演示一下"),
    ("展示以下", "展示一下"),
    # 真实麦克风长测样本：“取消导航”被 ZipFormer 提交为“取消刀”。
    # 该替换只命中完整领域短语，不把普通的“刀”泛化成导航控制。
    ("取消刀", "取消导航"),
    ("退出控住", "退出控制"),
    ("结束控住", "结束控制"),
    ("休民", "休眠"),
)


DEFAULT_CANONICAL_PHRASES: tuple[str, ...] = (
    "小智",
    "你好小智",
    "向前",
    "前进",
    "后退",
    "左转",
    "右转",
    "停下",
    "停止",
    "急停",
    "绕圈",
    "画圆",
    "正方形",
    "演示一下",
    "挥手",
    "把灯设成蓝色",
    "去门口",
    "取消导航",
    "退出控制",
    "结束控制",
    "休眠",
)


@dataclass(frozen=True)
class NormalizationMatch:
    source: str
    target: str
    score: float
    provider: str = "alias"

    def as_dict(self) -> dict:
        return {
            "source": self.source,
            "target": self.target,
            "score": round(self.score, 3),
            "provider": self.provider,
        }


@dataclass(frozen=True)
class NormalizationResult:
    original: str
    text: str
    matches: list[NormalizationMatch] = field(default_factory=list)

    @property
    def changed(self) -> bool:
        return self.original != self.text

    @property
    def confidence(self) -> float:
        if not self.matches:
            return 1.0 if not self.changed else 0.0
        return min(match.score for match in self.matches)

    def to_feedback_json(self) -> str:
        return json.dumps(
            {
                "status": "normalized",
                "reason": "command_normalized",
                "original": self.original,
                "normalized": self.text,
                "confidence": round(self.confidence, 3),
                "matches": [match.as_dict() for match in self.matches],
            },
            ensure_ascii=False,
        )


@dataclass(frozen=True)
class NormalizationRules:
    aliases: tuple[tuple[str, str], ...] = DEFAULT_ALIASES
    canonical_phrases: tuple[str, ...] = DEFAULT_CANONICAL_PHRASES

    @classmethod
    def defaults(cls) -> "NormalizationRules":
        return cls()

    @classmethod
    def from_yaml(cls, path: str | Path | None) -> "NormalizationRules":
        if not path:
            return cls.defaults()
        config_path = Path(path).expanduser()
        if not config_path.exists():
            return cls.defaults()

        data = _load_yaml_dict(config_path)
        aliases = list(DEFAULT_ALIASES)
        for item in data.get("aliases", []) or []:
            parsed = _parse_alias_item(item)
            if parsed is not None:
                aliases.append(parsed)

        canonical = list(DEFAULT_CANONICAL_PHRASES)
        for phrase in data.get("canonical_phrases", []) or []:
            value = str(phrase).strip()
            if value and value not in canonical:
                canonical.append(value)

        return cls(tuple(_dedupe_pairs(aliases)), tuple(_dedupe(canonical)))


def _load_yaml_dict(path: Path) -> dict:
    text = path.read_text(encoding="utf-8")
    try:
        import yaml

        loaded = yaml.safe_load(text) or {}
        return loaded if isinstance(loaded, dict) else {}
    except Exception:
        return _load_simple_yaml(text)


def _load_simple_yaml(text: str) -> dict:
    """解析本项目命令词表用到的 YAML 子集。

    支持：
      aliases:
        - source: 钱进
          target: 前进
        - [作转, 左转]
      canonical_phrases:
        - 前进
    """

    result: dict[str, list] = {"aliases": [], "canonical_phrases": []}
    section = ""
    pending_alias: dict[str, str] | None = None
    for raw in text.splitlines():
        stripped = raw.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if stripped in {"aliases:", "canonical_phrases:"}:
            if pending_alias:
                result["aliases"].append(pending_alias)
                pending_alias = None
            section = stripped[:-1]
            continue
        if section == "aliases":
            if stripped.startswith("- [") and stripped.endswith("]"):
                inside = stripped[3:-1]
                parts = [part.strip().strip("'\"") for part in inside.split(",", 1)]
                if len(parts) == 2:
                    result["aliases"].append(parts)
                continue
            if stripped.startswith("- source:"):
                if pending_alias:
                    result["aliases"].append(pending_alias)
                pending_alias = {"source": stripped.split(":", 1)[1].strip().strip("'\"")}
                continue
            if stripped.startswith("target:") and pending_alias is not None:
                pending_alias["target"] = stripped.split(":", 1)[1].strip().strip("'\"")
                continue
        elif section == "canonical_phrases" and stripped.startswith("- "):
            result["canonical_phrases"].append(stripped[2:].strip().strip("'\""))
    if pending_alias:
        result["aliases"].append(pending_alias)
    return result


def _parse_alias_item(item) -> tuple[str, str] | None:
    if isinstance(item, dict):
        source = str(item.get("source", "")).strip()
        target = str(item.get("target", "")).strip()
    elif isinstance(item, (list, tuple)) and len(item) == 2:
        source = str(item[0]).strip()
        target = str(item[1]).strip()
    else:
        return None
    if not source or not target:
        return None
    return (source, target)


def _dedupe(values: Iterable[str]) -> list[str]:
    seen = set()
    result = []
    for value in values:
        if value in seen:
            continue
        seen.add(value)
        result.append(value)
    return result


def _dedupe_pairs(values: Iterable[tuple[str, str]]) -> list[tuple[str, str]]:
    seen = set()
    result = []
    for source, target in values:
        key = (source, target)
        if key in seen:
            continue
        seen.add(key)
        result.append(key)
    return result


class SimilarityScorer(Protocol):
    name: str

    def ratio(self, left: str, right: str) -> float:
        ...


class DifflibScorer:
    name = "difflib"

    def ratio(self, left: str, right: str) -> float:
        return SequenceMatcher(None, left, right).ratio()


class RapidFuzzScorer:
    name = "rapidfuzz"

    def __init__(self):
        try:
            from rapidfuzz import fuzz
        except Exception as error:  # pragma: no cover - 依赖缺失时走 DifflibScorer
            raise RuntimeError("rapidfuzz is not available") from error
        self._fuzz = fuzz

    def ratio(self, left: str, right: str) -> float:
        return float(self._fuzz.ratio(left, right)) / 100.0


def default_scorer() -> SimilarityScorer:
    try:
        return RapidFuzzScorer()
    except RuntimeError:
        return DifflibScorer()


class CommandNormalizer:
    """规范化 ASR 文本，尽量只修控制命令，不改普通聊天内容。"""

    def __init__(
        self,
        *,
        scorer: SimilarityScorer | None = None,
        fuzzy_threshold: float = 0.82,
        rules: NormalizationRules | None = None,
        rules_path: str | Path | None = None,
    ):
        self._scorer = scorer or default_scorer()
        self._fuzzy_threshold = fuzzy_threshold
        self._rules = rules or NormalizationRules.from_yaml(rules_path)

    def normalize(self, text: str) -> NormalizationResult:
        original = text.strip()
        if not original:
            return NormalizationResult(original, original, [])

        normalized = _PUNCTUATION.sub("", original)
        matches: list[NormalizationMatch] = []

        normalized, alias_matches = self._apply_aliases(normalized)
        matches.extend(alias_matches)

        normalized, fuzzy_matches = self._apply_fuzzy_phrases(normalized)
        matches.extend(fuzzy_matches)

        return NormalizationResult(original, normalized, matches)

    def _apply_aliases(self, text: str) -> tuple[str, list[NormalizationMatch]]:
        matches: list[NormalizationMatch] = []
        result = text
        # 按长短从前到后匹配，避免“演示一下”先被“一下”之类短词扰动。
        for source, target in sorted(self._rules.aliases, key=lambda item: len(item[0]), reverse=True):
            if source in result:
                result = result.replace(source, target)
                matches.append(NormalizationMatch(source, target, 1.0, "alias"))
        return result, matches

    def _apply_fuzzy_phrases(self, text: str) -> tuple[str, list[NormalizationMatch]]:
        result = text
        matches: list[NormalizationMatch] = []
        for phrase in self._rules.canonical_phrases:
            if phrase in result:
                continue
            candidate = self._best_window(result, phrase)
            if candidate is None:
                continue
            source, score = candidate
            if score < self._fuzzy_threshold:
                continue
            # 长度 2 的中文词只有 1 个字相同也可能高误报；这里要求至少首/尾有一处相同。
            if len(phrase) <= 2 and source[0] != phrase[0] and source[-1] != phrase[-1]:
                continue
            result = result.replace(source, phrase, 1)
            matches.append(
                NormalizationMatch(source, phrase, score, self._scorer.name)
            )
        return result, matches

    def _best_window(self, text: str, phrase: str) -> tuple[str, float] | None:
        if len(text) < 2:
            return None
        best_source = ""
        best_score = 0.0
        lengths = range(max(2, len(phrase) - 1), min(len(text), len(phrase) + 1) + 1)
        for length in lengths:
            for start in range(0, len(text) - length + 1):
                source = text[start : start + length]
                if not self._looks_command_like(source, phrase):
                    continue
                score = self._scorer.ratio(source, phrase)
                if score > best_score:
                    best_source = source
                    best_score = score
        if not best_source:
            return None
        return best_source, best_score

    @staticmethod
    def _looks_command_like(source: str, phrase: str) -> bool:
        if source == phrase:
            return False
        shared = set(source) & set(phrase)
        if shared:
            return True
        # 常见同音/近音错词在 alias 表里已经覆盖；无共享字符时不做泛化替换，避免误伤聊天。
        return False


def normalize_command_text(text: str) -> NormalizationResult:
    return CommandNormalizer().normalize(text)
