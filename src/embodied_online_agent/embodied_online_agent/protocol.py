import json
from dataclasses import dataclass, field
from typing import List

from .types import ActionCommand


@dataclass
class ProtocolEvents:
    speech: List[str] = field(default_factory=list)
    actions: List[ActionCommand] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)


class TaggedStreamParser:
    """增量解析 LLM 的 <speech>/<action> 协议。

    speech 可以边到达边送 TTS；action 必须等完整闭标签和合法 JSON 后才对外暴露，
    防止半个 token 流形成不完整机器人指令。
    """

    SPEECH_OPEN = "<speech>"
    SPEECH_CLOSE = "</speech>"
    ACTION_OPEN = "<action>"
    ACTION_CLOSE = "</action>"

    def __init__(self):
        self.buffer = ""
        self.state = "outside"

    def feed(self, chunk: str) -> ProtocolEvents:
        events = ProtocolEvents()
        self.buffer += chunk
        while self.buffer:
            if self.state == "outside":
                if self._enter_next_tag():
                    continue
                keep = max(len(self.SPEECH_OPEN), len(self.ACTION_OPEN)) - 1
                if len(self.buffer) > keep:
                    self.buffer = self.buffer[-keep:]
                break

            if self.state == "speech":
                index = self.buffer.find(self.SPEECH_CLOSE)
                if index >= 0:
                    if index:
                        events.speech.append(self.buffer[:index])
                    self.buffer = self.buffer[index + len(self.SPEECH_CLOSE) :]
                    self.state = "outside"
                    continue
                safe = len(self.buffer) - len(self.SPEECH_CLOSE) + 1
                if safe > 0:
                    events.speech.append(self.buffer[:safe])
                    self.buffer = self.buffer[safe:]
                break

            index = self.buffer.find(self.ACTION_CLOSE)
            if index < 0:
                break
            raw = self.buffer[:index].strip()
            self.buffer = self.buffer[index + len(self.ACTION_CLOSE) :]
            self.state = "outside"
            try:
                payload = json.loads(raw)
                if not isinstance(payload, dict) or not isinstance(payload.get("name"), str):
                    raise ValueError("action requires a string name")
                arguments = payload.get("arguments", {})
                if not isinstance(arguments, dict):
                    raise ValueError("action arguments must be an object")
                events.actions.append(ActionCommand(payload["name"], arguments))
            except (ValueError, TypeError, json.JSONDecodeError) as exc:
                events.errors.append(f"invalid action: {exc}")
        return events

    def finish(self) -> ProtocolEvents:
        events = self.feed("")
        if self.state != "outside" or self.buffer.strip():
            events.errors.append("incomplete tagged response")
        return events

    def _enter_next_tag(self) -> bool:
        speech = self.buffer.find(self.SPEECH_OPEN)
        action = self.buffer.find(self.ACTION_OPEN)
        candidates = [(i, state, tag) for i, state, tag in (
            (speech, "speech", self.SPEECH_OPEN),
            (action, "action", self.ACTION_OPEN),
        ) if i >= 0]
        if not candidates:
            return False
        index, state, tag = min(candidates, key=lambda item: item[0])
        self.buffer = self.buffer[index + len(tag) :]
        self.state = state
        return True


class SentenceChunker:
    """按中文标点或长度切句，在首音频延迟与 TTS 调用次数之间折中。"""

    def __init__(self, max_chars: int = 32):
        self.max_chars = max_chars
        self.buffer = ""
        self.boundaries = set("。！？!?；;，,\n")

    def feed(self, text: str) -> List[str]:
        chunks: List[str] = []
        for char in text:
            self.buffer += char
            if char in self.boundaries or len(self.buffer) >= self.max_chars:
                value = self.buffer.strip()
                if value:
                    chunks.append(value)
                self.buffer = ""
        return chunks

    def finish(self) -> List[str]:
        value = self.buffer.strip()
        self.buffer = ""
        return [value] if value else []
