import json
import os
import threading
from pathlib import Path
from typing import Dict, List


class ConversationMemory:
    """Bounded, crash-safe JSON conversation memory."""

    def __init__(self, path: str, max_turns: int = 10):
        self.path = Path(os.path.expanduser(path))
        self.max_messages = max(1, max_turns) * 2
        self._lock = threading.RLock()
        self._messages: List[Dict[str, str]] = []
        self.load()

    def load(self) -> List[Dict[str, str]]:
        with self._lock:
            if not self.path.exists():
                self._messages = []
                return []
            try:
                raw = json.loads(self.path.read_text(encoding="utf-8"))
                self._messages = [
                    {"role": item["role"], "content": item["content"]}
                    for item in raw
                    if item.get("role") in {"user", "assistant"}
                    and isinstance(item.get("content"), str)
                ][-self.max_messages :]
            except (OSError, ValueError, TypeError, KeyError):
                self._messages = []
            return list(self._messages)

    def messages(self) -> List[Dict[str, str]]:
        with self._lock:
            return [dict(item) for item in self._messages]

    def prompt_messages(
        self, system_content: str, user_content: str
    ) -> List[Dict[str, str]]:
        """构造稳定的 system→history→user 前缀，供 llama.cpp KV cache 复用。"""
        with self._lock:
            history = [dict(item) for item in self._messages]
        return [
            {"role": "system", "content": system_content},
            *history,
            {"role": "user", "content": user_content},
        ]

    def append_turn(
        self,
        user_text: str,
        assistant_text: str,
        *,
        model_output: str | None = None,
    ) -> None:
        with self._lock:
            # 发送给模型的历史尽量保存模型原始协议文本，使下一次请求与 llama.cpp
            # slot 中的已生成 token 保持一致，从而延长 KV cache 公共前缀。
            assistant_content = (
                model_output.strip()
                if model_output is not None and model_output.strip()
                else assistant_text
            )
            self._messages.extend(
                [
                    {"role": "user", "content": user_text},
                    {"role": "assistant", "content": assistant_content},
                ]
            )
            self._messages = self._messages[-self.max_messages :]
            self._save()

    def clear(self) -> None:
        with self._lock:
            self._messages = []
            self._save()

    def _save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(self._messages, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        os.replace(temporary, self.path)
