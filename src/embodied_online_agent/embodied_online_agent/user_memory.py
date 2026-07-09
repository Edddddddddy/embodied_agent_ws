"""Per-user memory primitives for speaker-aware voice control."""

import json
import os
import re
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional


_UNKNOWN_SPEAKER_ID = "unknown"
_SAFE_ID_RE = re.compile(r"[^a-zA-Z0-9_\-\u4e00-\u9fff]+")


@dataclass(frozen=True)
class SpeakerIdentity:
    """声纹识别结果。

    Agent 只依赖这个稳定的小结构，不直接依赖 sherpa-onnx/SpeechBrain 等具体模型。
    这样真实声纹 provider、mock provider、未来多说话人 diarization 都可以复用同一接口。
    """

    speaker_id: str = _UNKNOWN_SPEAKER_ID
    confidence: float = 0.0
    enrolled: bool = False
    model: str = "unknown"
    display_name: str = ""
    updated_at: float = 0.0

    @property
    def usable(self) -> bool:
        return bool(
            self.enrolled
            and self.speaker_id
            and self.speaker_id != _UNKNOWN_SPEAKER_ID
        )

    def as_dict(self) -> dict:
        return {
            "speaker_id": self.speaker_id,
            "confidence": round(float(self.confidence), 4),
            "enrolled": bool(self.enrolled),
            "model": self.model,
            "display_name": self.display_name,
            "updated_at": self.updated_at,
        }

    @classmethod
    def from_json(cls, payload: str, *, min_confidence: float = 0.55) -> "SpeakerIdentity":
        try:
            raw = json.loads(payload)
        except (json.JSONDecodeError, TypeError):
            return cls(updated_at=time.time())
        if not isinstance(raw, dict):
            return cls(updated_at=time.time())
        speaker_id = (
            str(raw.get("speaker_id") or _UNKNOWN_SPEAKER_ID).strip()
            or _UNKNOWN_SPEAKER_ID
        )
        confidence = _as_float(raw.get("confidence"), 0.0)
        enrolled = bool(raw.get("enrolled", False)) and confidence >= min_confidence
        return cls(
            speaker_id=speaker_id if enrolled else _UNKNOWN_SPEAKER_ID,
            confidence=confidence,
            enrolled=enrolled,
            model=str(raw.get("model") or "unknown"),
            display_name=str(raw.get("display_name") or "").strip(),
            updated_at=time.time(),
        )


@dataclass
class MemoryCommand:
    kind: str
    value: Any = None


@dataclass
class UserProfile:
    speaker_id: str
    display_name: str = ""
    preferences: Dict[str, Any] = field(default_factory=dict)
    command_counts: Dict[str, int] = field(default_factory=dict)
    recent_interactions: List[Dict[str, Any]] = field(default_factory=list)
    corrections: List[Dict[str, Any]] = field(default_factory=list)
    updated_at: float = 0.0

    def as_dict(self) -> dict:
        return {
            "speaker_id": self.speaker_id,
            "display_name": self.display_name,
            "preferences": dict(self.preferences),
            "command_counts": dict(self.command_counts),
            "recent_interactions": list(self.recent_interactions),
            "corrections": list(self.corrections),
            "updated_at": self.updated_at,
        }

    @classmethod
    def from_dict(cls, raw: dict, speaker_id: str) -> "UserProfile":
        return cls(
            speaker_id=str(raw.get("speaker_id") or speaker_id),
            display_name=str(raw.get("display_name") or ""),
            preferences=dict(raw.get("preferences") or {}),
            command_counts={
                str(key): int(value)
                for key, value in dict(raw.get("command_counts") or {}).items()
                if isinstance(value, (int, float))
            },
            recent_interactions=[
                item
                for item in list(raw.get("recent_interactions") or [])
                if isinstance(item, dict)
            ],
            corrections=[
                item
                for item in list(raw.get("corrections") or [])
                if isinstance(item, dict)
            ],
            updated_at=_as_float(raw.get("updated_at"), 0.0),
        )


class UserMemoryStore:
    """按说话人保存用户画像与行为习惯。

    ConversationMemory 适合“最近几轮对话”；这里保存的是更稳定的用户画像。
    两者分开，可以避免把长时间聊天原文无限塞进 prompt，也方便按用户清除隐私数据。
    """

    def __init__(self, root_dir: str, *, max_recent: int = 8):
        self.root_dir = Path(os.path.expanduser(root_dir))
        self.max_recent = max(1, int(max_recent))
        self._lock = threading.RLock()

    def profile(self, identity: SpeakerIdentity | str | None) -> UserProfile:
        speaker_id = self._speaker_id(identity)
        with self._lock:
            return self._load_profile(speaker_id)

    def enroll(
        self, identity: SpeakerIdentity | str | None, display_name: str
    ) -> UserProfile:
        speaker_id = self._speaker_id(identity)
        with self._lock:
            profile = self._load_profile(speaker_id)
            profile.display_name = display_name.strip() or profile.display_name
            profile.updated_at = time.time()
            self._save_profile(profile)
            return profile

    def clear(self, identity: SpeakerIdentity | str | None) -> None:
        speaker_id = self._speaker_id(identity)
        with self._lock:
            path = self._profile_path(speaker_id)
            if path.exists():
                path.unlink()

    def set_preference(
        self, identity: SpeakerIdentity | str | None, key: str, value: Any
    ) -> UserProfile:
        speaker_id = self._speaker_id(identity)
        with self._lock:
            profile = self._load_profile(speaker_id)
            profile.preferences[key] = value
            profile.updated_at = time.time()
            self._save_profile(profile)
            return profile

    def record_interaction(
        self,
        identity: SpeakerIdentity | str | None,
        *,
        user_text: str,
        assistant_text: str = "",
        actions: Optional[List[dict]] = None,
        success: Optional[bool] = None,
    ) -> UserProfile:
        speaker_id = self._speaker_id(identity)
        with self._lock:
            profile = self._load_profile(speaker_id)
            now = time.time()
            for action in actions or []:
                name = str(action.get("name") or "")
                if name:
                    profile.command_counts[name] = profile.command_counts.get(name, 0) + 1
            profile.recent_interactions.append(
                {
                    "ts": now,
                    "user": user_text,
                    "assistant": assistant_text,
                    "actions": actions or [],
                    "success": success,
                }
            )
            profile.recent_interactions = profile.recent_interactions[-self.max_recent :]
            profile.updated_at = now
            self._save_profile(profile)
            return profile

    def prompt_summary(
        self, identity: SpeakerIdentity | str | None, *, max_chars: int = 360
    ) -> str:
        speaker_id = self._speaker_id(identity)
        if speaker_id == _UNKNOWN_SPEAKER_ID:
            return ""
        profile = self.profile(speaker_id)
        parts = []
        name = profile.display_name or speaker_id
        parts.append(f"当前识别用户：{name}（speaker_id={speaker_id}）。")
        if profile.preferences:
            preference_text = "；".join(
                f"{key}={value}" for key, value in sorted(profile.preferences.items())
            )
            parts.append(f"用户偏好：{preference_text}。")
        if profile.command_counts:
            top = sorted(
                profile.command_counts.items(), key=lambda item: item[1], reverse=True
            )[:3]
            parts.append(
                "常用动作：" + "，".join(f"{name}×{count}" for name, count in top) + "。"
            )
        summary = "\n".join(parts).strip()
        # prompt 注入必须有上限：用户画像是辅助上下文，不应该挤占动作格式约束。
        return summary[: max(0, int(max_chars))]

    def _load_profile(self, speaker_id: str) -> UserProfile:
        path = self._profile_path(speaker_id)
        if not path.exists():
            return UserProfile(speaker_id=speaker_id, updated_at=time.time())
        try:
            raw = json.loads(path.read_text(encoding="utf-8"))
            if isinstance(raw, dict):
                return UserProfile.from_dict(raw, speaker_id)
        except (OSError, ValueError, TypeError):
            pass
        return UserProfile(speaker_id=speaker_id, updated_at=time.time())

    def _save_profile(self, profile: UserProfile) -> None:
        self.root_dir.mkdir(parents=True, exist_ok=True)
        path = self._profile_path(profile.speaker_id)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(profile.as_dict(), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        os.replace(temporary, path)

    def _profile_path(self, speaker_id: str) -> Path:
        safe = _SAFE_ID_RE.sub("_", speaker_id or _UNKNOWN_SPEAKER_ID).strip("._")
        return self.root_dir / f"{safe or _UNKNOWN_SPEAKER_ID}.json"

    @staticmethod
    def _speaker_id(identity: SpeakerIdentity | str | None) -> str:
        if isinstance(identity, SpeakerIdentity):
            return identity.speaker_id if identity.usable else _UNKNOWN_SPEAKER_ID
        if isinstance(identity, str) and identity.strip():
            return identity.strip()
        return _UNKNOWN_SPEAKER_ID


def parse_memory_command(text: str) -> Optional[MemoryCommand]:
    compact = _normalize_text(text)
    if not compact:
        return None
    if any(word in compact for word in ("我是谁", "现在是谁", "识别到谁")):
        return MemoryCommand("whoami")
    if any(word in compact for word in ("清除我的记忆", "删除我的记忆", "忘记我")):
        return MemoryCommand("clear")
    if any(word in compact for word in ("录入我的声纹", "注册声纹", "记住我的声音")):
        return MemoryCommand("enroll_request")

    name = _extract_name(compact)
    if name:
        return MemoryCommand("enroll_name", name)

    preference = _extract_preference(compact)
    if preference:
        return MemoryCommand("preference", preference)
    return None


def _extract_name(compact: str) -> str:
    for prefix in ("记住我我是", "我是", "我叫"):
        if prefix in compact:
            candidate = compact.split(prefix, 1)[1]
            candidate = candidate[:12]
            return candidate.strip("，。,.!！?？ ")[:12]
    return ""


def _extract_preference(compact: str) -> Optional[dict]:
    if "喜欢慢一点" in compact or "速度慢一点" in compact or compact == "慢一点":
        return {"movement_speed": "slow"}
    if "喜欢快一点" in compact or "速度快一点" in compact or compact == "快一点":
        return {"movement_speed": "fast"}
    match = re.search(
        r"(?:以后)?(?:前进|向前走|向前)?默认(?:前进|向前走|向前)?([0-9一二两三四五六七八九十]+)秒",
        compact,
    )
    if match:
        duration = _parse_small_number(match.group(1))
        if duration is not None:
            return {"default_move_duration_s": duration}
    match = re.search(
        r"(?:以后)?(?:左转|右转|转弯|转向)?默认(?:左转|右转|转弯|转向)?([0-9一二两三四五六七八九十百]+)度",
        compact,
    )
    if match:
        degrees = _parse_degree_number(match.group(1))
        if degrees is not None:
            return {"default_turn_degrees": degrees}
    return None


def _normalize_text(text: str) -> str:
    return "".join(ch for ch in text.strip() if ch not in " \t\r\n，。！？!?、,.；;：:")


def _parse_small_number(value: str) -> Optional[float]:
    if value.isdigit():
        number = int(value)
        return float(number) if 0 < number <= 10 else None
    mapping = {
        "一": 1,
        "二": 2,
        "两": 2,
        "三": 3,
        "四": 4,
        "五": 5,
        "六": 6,
        "七": 7,
        "八": 8,
        "九": 9,
        "十": 10,
    }
    if value in mapping:
        return float(mapping[value])
    return None


def _parse_degree_number(value: str) -> Optional[float]:
    if value.isdigit():
        number = int(value)
        return float(number) if 15 <= number <= 360 else None
    aliases = {
        "十五": 15,
        "三十": 30,
        "四十五": 45,
        "六十": 60,
        "九十": 90,
        "一百八十": 180,
        "三百六十": 360,
    }
    if value in aliases:
        return float(aliases[value])
    parsed = _parse_chinese_integer(value)
    return float(parsed) if parsed is not None and 15 <= parsed <= 360 else None


def _parse_chinese_integer(value: str) -> Optional[int]:
    digits = {
        "零": 0,
        "一": 1,
        "二": 2,
        "两": 2,
        "三": 3,
        "四": 4,
        "五": 5,
        "六": 6,
        "七": 7,
        "八": 8,
        "九": 9,
    }
    if not value:
        return None
    if value in digits:
        return digits[value]
    if "百" in value:
        left, right = value.split("百", 1)
        hundreds = digits.get(left, 1 if not left else None)
        if hundreds is None:
            return None
        remainder = _parse_chinese_integer(right) if right else 0
        return hundreds * 100 + remainder if remainder is not None else None
    if "十" in value:
        left, right = value.split("十", 1)
        tens = digits.get(left, 1 if not left else None)
        ones = digits.get(right, 0 if not right else None)
        if tens is None or ones is None:
            return None
        return tens * 10 + ones
    return None


def _as_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default
