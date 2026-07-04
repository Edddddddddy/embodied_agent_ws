"""解析外部 KWS/唤醒节点发来的 ROS 文本事件。

声学唤醒模型不应直接依赖 online/offline Agent 的内部类。约定它们只需要往
`/agent/wake_event_input` 发布一个短 JSON，例如：

    {"kind": "wake", "provider": "sherpa_kws", "transcript": "小智"}
    {"kind": "sleep", "provider": "openwakeword"}

为了手工调试方便，也兼容纯文本 `wake` / `sleep`。
"""

from __future__ import annotations

import json
from dataclasses import dataclass


@dataclass(frozen=True)
class ExternalWakeEvent:
    kind: str
    provider: str = "external"
    transcript: str = ""


def parse_external_wake_event(payload: str) -> ExternalWakeEvent | None:
    text = payload.strip()
    if not text:
        return None
    if text.lower() in {"wake", "sleep"}:
        return ExternalWakeEvent(text.lower())
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        return None
    if not isinstance(data, dict):
        return None
    kind = str(data.get("kind", "")).strip().lower()
    if kind not in {"wake", "sleep"}:
        return None
    provider = str(data.get("provider", "external")).strip() or "external"
    transcript = str(data.get("transcript", "")).strip()
    return ExternalWakeEvent(kind, provider, transcript)
