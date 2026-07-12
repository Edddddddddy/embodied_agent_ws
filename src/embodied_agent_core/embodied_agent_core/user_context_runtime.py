"""说话人身份、用户画像和 turn 级上下文的一致性边界。"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Mapping

from .memory_command_service import MemoryCommandResult, MemoryCommandService
from .user_memory import (
    LowConfidenceSpeakerError,
    SpeakerIdentity,
    UserMemoryStore,
)


@dataclass(frozen=True)
class UserContextSnapshot:
    """命令进入执行链路时冻结的用户上下文。

    声纹识别可能在 LLM/TTS 或长动作执行期间更新；快照保证同一个 turn 不会读取 A 的
    偏好却把结果写入 B 的画像。
    """

    identity: SpeakerIdentity
    preferences: Mapping[str, Any]
    prompt_summary: str = ""

    def system_prompt(self, base_prompt: str) -> str:
        if not self.prompt_summary:
            return base_prompt
        return (
            base_prompt
            + "\n\n[用户画像记忆]\n"
            + self.prompt_summary
            + "\n请仅把用户画像作为偏好参考，所有动作仍必须遵守输出格式和安全限幅。"
        )


class UserContextRuntime:
    """用户身份与画像的唯一运行时拥有者，不依赖 ROS。"""

    def __init__(
        self,
        store: UserMemoryStore,
        *,
        command_service: MemoryCommandService | None = None,
    ):
        self._store = store
        self._commands = command_service or MemoryCommandService(store)
        self._identity = SpeakerIdentity()
        self._lock = threading.RLock()

    @property
    def identity(self) -> SpeakerIdentity:
        with self._lock:
            return self._identity

    def update_identity(self, identity: SpeakerIdentity) -> None:
        with self._lock:
            self._identity = identity

    def snapshot(self) -> UserContextSnapshot:
        with self._lock:
            identity = self._identity
            if not identity.usable:
                return UserContextSnapshot(identity, MappingProxyType({}))
            profile = self._store.profile(identity)
            summary = self._store.prompt_summary(identity)
            return UserContextSnapshot(
                identity,
                MappingProxyType(dict(profile.preferences)),
                summary,
            )

    def handle_command(self, command: str) -> MemoryCommandResult | None:
        with self._lock:
            result = self._commands.handle(command, self._identity)
            if result is not None:
                self._identity = result.identity
            return result

    def clear_current(self) -> bool:
        with self._lock:
            identity = self._identity
        try:
            self._store.clear(identity)
            return True
        except (LowConfidenceSpeakerError, OSError):
            return False

    def record_interaction(
        self,
        context: UserContextSnapshot,
        *,
        user_text: str,
        assistant_text: str,
        actions: list[dict],
        success: bool,
    ) -> bool:
        try:
            self._store.record_interaction(
                context.identity,
                user_text=user_text,
                assistant_text=assistant_text,
                actions=actions,
                success=success,
            )
            return True
        except (LowConfidenceSpeakerError, OSError):
            # unknown/低置信度身份只允许读取空上下文，绝不能落盘到共享画像。
            return False
