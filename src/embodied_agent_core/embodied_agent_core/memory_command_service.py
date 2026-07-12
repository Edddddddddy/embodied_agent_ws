"""在线/离线 Agent 共用的用户记忆命令深模块。"""

from __future__ import annotations

import time
from dataclasses import dataclass

from .user_memory import (
    LowConfidenceSpeakerError,
    SpeakerIdentity,
    UserMemoryStore,
    parse_memory_command,
)


@dataclass(frozen=True)
class SpeakerEnrollRequest:
    speaker_id: str
    display_name: str
    samples_required: int = 3


@dataclass(frozen=True)
class MemoryCommandResult:
    response: str
    identity: SpeakerIdentity
    enroll_request: SpeakerEnrollRequest | None = None


class MemoryCommandService:
    """解析、执行并记录记忆命令；节点只负责 ROS 发布与 TTS。

    接口刻意只暴露 ``handle``：查询、录入、偏好变更、删除、清空和交互记录的
    排序约束全部留在实现内部，避免 online/offline 调用者再次复制状态分支。
    """

    def __init__(self, store: UserMemoryStore, *, clock=time.time):
        self._store = store
        self._clock = clock

    def handle(
        self, command: str, identity: SpeakerIdentity
    ) -> MemoryCommandResult | None:
        parsed = parse_memory_command(command)
        if parsed is None:
            return None

        enroll_request = None
        should_record = parsed.kind != "clear"
        if parsed.kind == "whoami":
            if identity.usable:
                profile = self._store.profile(identity)
                name = profile.display_name or identity.display_name or identity.speaker_id
                response = f"我识别到当前用户是：{name}。"
            else:
                response = "我还没有可靠识别到当前用户，可以先说“记住我，我是某某”。"
        elif parsed.kind == "query_preferences":
            if identity.usable:
                profile = self._store.profile(identity)
                response = (
                    "你的当前偏好："
                    + "，".join(
                        f"{key}={value}"
                        for key, value in sorted(profile.preferences.items())
                    )
                    if profile.preferences
                    else "当前没有保存个人偏好。"
                )
            else:
                response = "我还没有可靠识别当前用户，无法查询个人偏好。"
        elif parsed.kind == "clear":
            if identity.usable:
                self._store.clear(identity)
                response = "已清除当前用户的本地行为记忆。"
            else:
                response = "我还没有可靠识别当前用户，无法清除个人记忆。"
        elif parsed.kind == "enroll_request":
            if identity.usable:
                enroll_request = SpeakerEnrollRequest(
                    identity.speaker_id,
                    identity.display_name or identity.speaker_id,
                )
                response = "已开始声纹录入，请连续说三句短句用于采集样本。"
            else:
                response = "请先说“记住我，我是某某”，我会用这个名字开始声纹录入。"
        elif parsed.kind == "enroll_name":
            if not identity.usable:
                # 文本录入只用于演示兜底；真实身份仍由声纹 sidecar 后续覆盖。
                identity = SpeakerIdentity(
                    speaker_id=str(parsed.value),
                    confidence=1.0,
                    enrolled=True,
                    model="text-enroll-fallback",
                    display_name=str(parsed.value),
                    updated_at=self._clock(),
                )
            profile = self._store.enroll(identity, str(parsed.value))
            enroll_request = SpeakerEnrollRequest(identity.speaker_id, profile.display_name)
            response = (
                f"好的，我记住你是 {profile.display_name or profile.speaker_id}。"
                "如果已启动声纹 sidecar，请继续说三句短句完成样本采集。"
            )
        elif parsed.kind == "preference":
            if not identity.usable:
                response = "我还没有可靠识别当前用户，先说“记住我，我是某某”后再记录偏好。"
            else:
                profile = self._store.profile(identity)
                for key, value in dict(parsed.value).items():
                    profile = self._store.set_preference(identity, key, value)
                response = "已记录你的偏好：" + "，".join(
                    f"{key}={value}" for key, value in sorted(profile.preferences.items())
                )
        elif parsed.kind == "delete_preference":
            if not identity.usable:
                response = "我还没有可靠识别当前用户，无法删除个人偏好。"
            else:
                key = str(parsed.value)
                profile = self._store.remove_preference(identity, key)
                response = f"已删除偏好：{key}。"
                if profile.preferences:
                    response += "当前保留：" + "，".join(
                        f"{name}={value}"
                        for name, value in sorted(profile.preferences.items())
                    )
        else:
            return None

        if should_record:
            try:
                self._store.record_interaction(
                    identity,
                    user_text=command,
                    assistant_text=response,
                    actions=[],
                    success=True,
                )
            except (LowConfidenceSpeakerError, OSError):
                # 未可靠识别用户时仍可回答，但绝不能把记录写到 unknown 画像。
                # 记录属于辅助证据，磁盘暂时不可写也不应让控制会话失败。
                pass
        return MemoryCommandResult(response, identity, enroll_request)
