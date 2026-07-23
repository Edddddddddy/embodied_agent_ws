"""短语音命令补全。

真实麦克风场景里，ASR final 偶尔会漏掉尾部数字或量词，例如把“左转90度”
识别成“左转”。这里不尝试猜复杂语义，只对少量安全、明确的机器人控制短命令
补默认槽位，让连续语音演示不中断。
"""

from __future__ import annotations

import re
from dataclasses import dataclass


_PUNCTUATION = re.compile(r"[，。！？!?\s、,.；;：:]")

# 只补“缺少默认槽位”的确定性短命令。这里故意不用大模型再推理一次：
# 连续语音场景需要低延迟和可预测性，裸“左转/前进”映射成演示默认动作即可。
_COMPLETIONS = {
    "前进": "前进一秒",
    "向前": "前进一秒",
    "往前": "前进一秒",
    "后退": "后退一秒",
    "向后": "后退一秒",
    "往后": "后退一秒",
    "左转": "左转九十度",
    "向左转": "左转九十度",
    "右转": "右转九十度",
    "向右转": "右转九十度",
}


@dataclass(frozen=True)
class CompletionResult:
    original: str
    text: str
    reason: str = ""

    @property
    def changed(self) -> bool:
        return self.original != self.text

    def feedback_dict(self) -> dict:
        return {
            "status": "completed",
            "reason": self.reason or "completed_missing_slot",
            "original": self.original,
            "completed": self.text,
            "confidence": 1.0,
        }


class CommandCompleter:
    """只补全明确短控制命令，不处理普通聊天和安全停止命令。"""

    def __init__(self, *, enabled: bool = True):
        self._enabled = enabled

    def complete(self, text: str) -> CompletionResult:
        original = text.strip()
        if not self._enabled or not original:
            return CompletionResult(original, original)

        # 归一化仅移除标点和空白，避免把“不前进”之类自然语言误归为“前进”。
        normalized = _PUNCTUATION.sub("", original).lower()
        completed = _COMPLETIONS.get(normalized)
        if completed is None:
            return CompletionResult(original, original)
        return CompletionResult(original, completed, "completed_missing_slot")
