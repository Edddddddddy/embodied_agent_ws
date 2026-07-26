"""在线/离线 Agent 共用的 LLM 流式输出协议运行时。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Sequence

from .command_fallback import parse_fallback_actions, should_block_model_actions
from .protocol import SentenceChunker, TaggedStreamParser
from .types import ActionCommand


@dataclass(frozen=True)
class StreamingTurnResult:
    """一次模型流解析完成后的稳定结果，不暴露增量 parser 的内部状态。"""

    assistant_text: str
    model_output: str
    raw_output: str
    actions: tuple[ActionCommand, ...]
    model_actions: tuple[ActionCommand, ...]
    protocol_errors: tuple[str, ...]
    action_source: str


class StreamingTurnRuntime:
    """隐藏 tagged protocol、TTS 分句与动作选择策略。

    provider 只提供四个窄回调：首 token、文字增量、可合成句子和协议告警。
    在线 Agent 可把句子送入流式 TTS 队列，离线 Agent 可送入双缓冲管线；协议与
    安全策略因此只有一份权威实现。
    """

    FORMAT_ERROR_SPEECH = "抱歉，回复格式解析失败，请再说一次。"

    def __init__(
        self,
        *,
        max_chunk_chars: int,
        on_first_token: Callable[[], None],
        on_speech_delta: Callable[[str], None],
        on_speakable: Callable[[str], None],
        on_protocol_error: Callable[[str], None],
    ):
        self._parser = TaggedStreamParser()
        self._chunker = SentenceChunker(max_chunk_chars)
        self._on_first_token = on_first_token
        self._on_speech_delta = on_speech_delta
        self._on_speakable = on_speakable
        self._on_protocol_error = on_protocol_error
        self._first_token = True
        self._speech_parts: list[str] = []
        self._model_actions: list[ActionCommand] = []
        self._raw_output_parts: list[str] = []
        self._protocol_errors: list[str] = []
        self._finished = False

    def feed(self, token: str) -> None:
        if self._finished:
            raise RuntimeError("streaming turn is already finished")
        self._raw_output_parts.append(token)
        if self._first_token:
            self._on_first_token()
            self._first_token = False
        self._consume_events(self._parser.feed(token))

    def finish(
        self, user_text: str, *, allow_actions: bool = True
    ) -> StreamingTurnResult:
        if self._finished:
            raise RuntimeError("streaming turn is already finished")
        self._finished = True
        self._consume_events(self._parser.finish())

        # 模型格式损坏时仍给用户明确反馈，但绝不从残缺 action 标签推断机器人动作。
        if not self._speech_parts:
            self._accept_speech(self.FORMAT_ERROR_SPEECH)
        for sentence in self._chunker.finish():
            self._on_speakable(sentence)

        selected, source = self._select_actions(
            user_text, allow_actions=allow_actions
        )
        assistant_text = "".join(self._speech_parts).strip()
        raw_output = "".join(self._raw_output_parts).strip()
        model_output = (
            raw_output
            if raw_output and not self._protocol_errors
            else f"<speech>{assistant_text}</speech>"
        )
        return StreamingTurnResult(
            assistant_text=assistant_text,
            model_output=model_output,
            raw_output=raw_output,
            actions=tuple(selected),
            model_actions=tuple(self._model_actions),
            protocol_errors=tuple(self._protocol_errors),
            action_source=source,
        )

    def _consume_events(self, events) -> None:
        for delta in events.speech:
            self._accept_speech(delta)
        self._model_actions.extend(events.actions)
        for error in events.errors:
            self._protocol_errors.append(error)
            self._on_protocol_error(error)

    def _accept_speech(self, delta: str) -> None:
        self._speech_parts.append(delta)
        self._on_speech_delta(delta)
        for sentence in self._chunker.feed(delta):
            self._on_speakable(sentence)

    def _select_actions(
        self, user_text: str, *, allow_actions: bool
    ) -> tuple[Sequence[ActionCommand], str]:
        # 只有上游本地 NLU 明确认定为控制意图的 turn 才能授权动作。普通聊天或
        # RAG 问答即使诱导模型输出合法 action，也在 Python 应用边界 fail-closed；
        # C++ ActionGuard 是后续的第二道安全线。
        if not allow_actions:
            return (), "context_blocked"
        deterministic = parse_fallback_actions(user_text)
        if deterministic:
            return deterministic, "deterministic"
        if should_block_model_actions(user_text):
            return (), "blocked"
        if self._model_actions:
            return self._model_actions, "model"
        return (), "none"
