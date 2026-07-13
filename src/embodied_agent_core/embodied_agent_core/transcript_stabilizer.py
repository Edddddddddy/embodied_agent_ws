"""保守合并流式 ASR partial/final，修复控制槽位尾部被 final 截断的问题。"""

from __future__ import annotations

import re
import time
from dataclasses import dataclass


_PUNCTUATION = re.compile(r"[^0-9a-zA-Z\u4e00-\u9fff]")
_NUMBER = r"(?:\d+(?:\.\d+)?|[零〇一二两三四五六七八九十百点半]+)"
_COLOR_TAIL = re.compile(r"(?:设|设置)?(?:成|为)?(?:红|绿|蓝|黄|白)(?:色)?")
_QUANTITY_TAIL = re.compile(rf"(?:以?每秒{_NUMBER}米)?{_NUMBER}(?:秒|分钟|度|次|米)?")
_CONTROL_TAILS = {
    "导航",
    "控制",
    "自动模式",
    "手动模式",
    "门口",
    "客厅",
    "充电区",
    "起点",
    "书桌",
    "厨房",
}


def _normalize(text: str) -> str:
    return _PUNCTUATION.sub("", str(text)).lower()


def _is_safe_slot_tail(tail: str) -> bool:
    """只允许恢复可解释控制槽位，绝不把任意聊天 partial 覆盖 final。"""
    if not tail:
        return False
    if tail in _CONTROL_TAILS:
        return True
    if _COLOR_TAIL.fullmatch(tail):
        return True
    if _QUANTITY_TAIL.fullmatch(tail):
        # 裸数字至少要包含常见控制量，避免用普通序号覆盖云端最终文本。
        return any(marker in tail for marker in ("秒", "分钟", "度", "次", "米")) or tail in {
            "四十五", "九十", "一百八十", "三百六十", "45", "90", "180", "360"
        }
    return False


@dataclass(frozen=True)
class StabilizedTranscript:
    original_final: str
    text: str
    partial: str = ""
    reason: str = ""

    @property
    def recovered(self) -> bool:
        return self.text != self.original_final

    def feedback_dict(self) -> dict:
        return {
            "status": "asr_final_recovered",
            "reason": self.reason,
            "original_final": self.original_final,
            "recovered": self.text,
            "partial": self.partial,
        }


class TranscriptStabilizer:
    """维护单个 utterance 的最新 partial，并在 final 时做一次保守恢复。

    final 必须是最新 partial 的严格前缀，且多出的尾部必须是已知控制槽位。
    每次 finalize 都清空状态，避免上一句话污染下一句话。
    """

    def __init__(self, *, enabled: bool = True, max_age_s: float = 2.0, clock=time.monotonic):
        self._enabled = bool(enabled)
        self._max_age_s = max(0.0, float(max_age_s))
        self._clock = clock
        self._partial = ""
        self._partial_at = 0.0

    def observe_partial(self, text: str) -> None:
        partial = str(text).strip()
        if not self._enabled or not partial:
            return
        # 只保存模型最新判断；遇到互相矛盾的 partial 时不保留更长的旧猜测。
        self._partial = partial
        self._partial_at = self._clock()

    def finalize(self, text: str) -> StabilizedTranscript:
        final = str(text).strip()
        partial = self._partial
        partial_at = self._partial_at
        self.clear()
        if not self._enabled or not final or not partial:
            return StabilizedTranscript(final, final, partial)
        if self._max_age_s > 0.0 and self._clock() - partial_at > self._max_age_s:
            return StabilizedTranscript(final, final, partial)

        final_key = _normalize(final)
        partial_key = _normalize(partial)
        if not final_key or not partial_key.startswith(final_key) or len(partial_key) <= len(final_key):
            return StabilizedTranscript(final, final, partial)
        tail = partial_key[len(final_key) :]
        if not _is_safe_slot_tail(tail):
            return StabilizedTranscript(final, final, partial)
        return StabilizedTranscript(
            final,
            partial,
            partial,
            "partial_slot_tail_recovered",
        )

    def clear(self) -> None:
        self._partial = ""
        self._partial_at = 0.0
