"""聚合真人语音触发与 strict unknown-world 核心报告的 ROS-free 证据。"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass
import math
from typing import Mapping

from .showcase_session import SessionCommand, parse_session_command


VOICE_EVIDENCE_KIND = "voice_unknown_world_slam_nav_e2e"
CORE_EVIDENCE_KIND = "unknown_world_slam_nav_dynamic_replan"
# 这里有意镜像 WakeEvent.msg 的 wire 值而不导入 ROS 消息包，使 evaluator
# 在纯 pytest/CI 环境也能运行；CONTINUE 表示会话已由先前真人唤醒并仍有效。
WAKE_KIND_WAKE = 1
WAKE_KIND_CONTINUE = 2
ACCEPTED_WAKE_KINDS = frozenset({WAKE_KIND_WAKE, WAKE_KIND_CONTINUE})
WAKE_ASR_CORRELATION_NS = 5_000_000_000

_CORE_REQUIRED_CHECKS = frozenset(
    {
        "unknown_world_profile",
        "mission_sequence_present",
        "mission_completed",
        "mission_outcome_succeeded",
        "map_saved",
        "fresh_session_map",
        "map_quality",
        "frontier_complete",
        "localization_quality",
        "sampled_navigation",
        "nav2_lifecycle_active",
        "dynamic_navigation",
        "cmd_vel_observed",
        "robot_motion_observed",
        "final_task_motion_observed",
        "final_cmd_vel_fresh",
        "final_cmd_vel_zero",
    }
)
_CORE_REQUIRED_SECTIONS = (
    "map_quality",
    "frontier",
    "localization",
    "sampled_navigation",
    "dynamic_navigation",
)


@dataclass(frozen=True, slots=True)
class AudioMetricSnapshot:
    observed_at_ns: int
    speech: bool
    rms: float
    peak: int


@dataclass(frozen=True, slots=True)
class EndpointSnapshot:
    observed_at_ns: int
    event: str
    provider: str


@dataclass(frozen=True, slots=True)
class WakeSnapshot:
    observed_at_ns: int
    kind: int
    provider: str
    transcript: str
    command_known: bool
    command: str


@dataclass(frozen=True, slots=True)
class AsrFinalSnapshot:
    observed_at_ns: int
    text: str


def _strict_core_schema_v4_passed(report: Mapping[str, object]) -> bool:
    """复核核心报告本身，不能只相信一个可手写的 ``passed=true``。"""

    if type(report.get("schema_version")) is not int:
        return False
    if report.get("schema_version") != 4:
        return False
    if report.get("evidence_kind") != CORE_EVIDENCE_KIND:
        return False
    if report.get("passed") is not True:
        return False
    if not str(report.get("session_id", "")).strip():
        return False
    if type(report.get("session_start_ns")) is not int:
        return False
    if int(report["session_start_ns"]) <= 0:
        return False
    if type(report.get("mission_sequence")) is not int:
        return False
    if int(report["mission_sequence"]) <= 0:
        return False

    checks = report.get("checks")
    if not isinstance(checks, Mapping):
        return False
    if not _CORE_REQUIRED_CHECKS.issubset(checks):
        return False
    if not all(value is True for value in checks.values()):
        return False
    for section_name in _CORE_REQUIRED_SECTIONS:
        section = report.get(section_name)
        if not isinstance(section, Mapping) or section.get("passed") is not True:
            return False
    return True


class VoiceTriggerEvidenceWindow:
    """在一个明确的采集窗口内，把语音事实与核心任务结果合成单一证据。"""

    def __init__(self, *, window_started_at_ns: int) -> None:
        if window_started_at_ns < 0:
            raise ValueError("window_started_at_ns must be non-negative")
        self._window_started_at_ns = int(window_started_at_ns)
        self._audio: list[AudioMetricSnapshot] = []
        self._endpoints: list[EndpointSnapshot] = []
        self._wake_events: list[WakeSnapshot] = []
        self._asr_finals: list[AsrFinalSnapshot] = []

    def _inside_window(self, observed_at_ns: int) -> bool:
        return int(observed_at_ns) >= self._window_started_at_ns

    def record_audio(
        self,
        *,
        observed_at_ns: int,
        speech: bool,
        rms: float,
        peak: int,
    ) -> bool:
        if not math.isfinite(rms) or rms < 0.0 or peak < 0:
            raise ValueError("audio metrics must be finite and non-negative")
        if not self._inside_window(observed_at_ns):
            return False
        self._audio.append(
            AudioMetricSnapshot(
                observed_at_ns=int(observed_at_ns),
                speech=bool(speech),
                rms=float(rms),
                peak=int(peak),
            )
        )
        return True

    def record_endpoint(
        self,
        *,
        observed_at_ns: int,
        event: str,
        provider: str,
    ) -> bool:
        if event not in {"speech_started", "speech_ended"}:
            raise ValueError("endpoint event must be speech_started/speech_ended")
        if not self._inside_window(observed_at_ns):
            return False
        self._endpoints.append(
            EndpointSnapshot(
                observed_at_ns=int(observed_at_ns),
                event=event,
                provider=str(provider).strip(),
            )
        )
        return True

    def record_wake(
        self,
        *,
        observed_at_ns: int,
        kind: int,
        provider: str,
        transcript: str,
        command_known: bool,
        command: str,
    ) -> bool:
        if not self._inside_window(observed_at_ns):
            return False
        self._wake_events.append(
            WakeSnapshot(
                observed_at_ns=int(observed_at_ns),
                kind=int(kind),
                provider=str(provider).strip(),
                transcript=str(transcript).strip(),
                command_known=bool(command_known),
                command=str(command).strip(),
            )
        )
        return True

    def record_asr_final(self, *, observed_at_ns: int, text: str) -> bool:
        if not self._inside_window(observed_at_ns):
            return False
        self._asr_finals.append(
            AsrFinalSnapshot(
                observed_at_ns=int(observed_at_ns), text=str(text).strip()
            )
        )
        return True

    def _complete_endpoints(self) -> tuple[tuple[int, int], ...]:
        starts: list[int] = []
        complete: list[tuple[int, int]] = []
        for item in sorted(self._endpoints, key=lambda value: value.observed_at_ns):
            if item.event == "speech_started":
                starts.append(item.observed_at_ns)
            elif starts:
                complete.append((starts.pop(), item.observed_at_ns))
        return tuple(complete)

    def _has_real_audio(self, endpoint: tuple[int, int]) -> bool:
        return any(
            endpoint[0] <= sample.observed_at_ns <= endpoint[1]
            and sample.speech
            and sample.rms > 0.0
            and sample.peak > 0
            for sample in self._audio
        )

    def build_envelope(
        self,
        core_report: Mapping[str, object],
        *,
        agent_mode: str,
    ) -> dict[str, object]:
        if agent_mode not in {"offline", "online"}:
            raise ValueError("agent_mode must be offline or online")
        endpoints = self._complete_endpoints()
        automatic_final: AsrFinalSnapshot | None = None
        endpoint: tuple[int, int] | None = None
        matched_wake: WakeSnapshot | None = None
        first_automatic_candidate: (
            tuple[AsrFinalSnapshot, tuple[int, int]] | None
        ) = None
        for item in sorted(self._asr_finals, key=lambda value: value.observed_at_ns):
            if (
                parse_session_command(item.text)
                != SessionCommand.RUN_AUTOMATIC_MISSION
            ):
                continue
            candidates = [pair for pair in endpoints if pair[1] <= item.observed_at_ns]
            if candidates:
                # 多轮语音中使用离 final 最近的闭合端点，避免把早先 filler 的
                # 音量证据错误嫁接到真正的建图命令上。
                candidate_endpoint = max(candidates, key=lambda pair: pair[1])
                if first_automatic_candidate is None:
                    first_automatic_candidate = (item, candidate_endpoint)
                wake_candidates = [
                    wake
                    for wake in self._wake_events
                    if wake.kind in ACCEPTED_WAKE_KINDS
                    and bool(wake.provider)
                    and bool(wake.transcript)
                    and wake.command_known
                    and parse_session_command(wake.command)
                    == SessionCommand.RUN_AUTOMATIC_MISSION
                    and wake.observed_at_ns >= candidate_endpoint[1]
                    and abs(wake.observed_at_ns - item.observed_at_ns)
                    <= WAKE_ASR_CORRELATION_NS
                ]
                if wake_candidates:
                    automatic_final = item
                    endpoint = candidate_endpoint
                    matched_wake = min(
                        wake_candidates,
                        key=lambda wake: abs(
                            wake.observed_at_ns - item.observed_at_ns
                        ),
                    )
                    break
        if automatic_final is None and first_automatic_candidate is not None:
            automatic_final, endpoint = first_automatic_candidate
        real_audio = bool(endpoint and self._has_real_audio(endpoint))
        checks = {
            "real_audio_observed": real_audio,
            "endpoint_observed": bool(endpoints),
            # 不是“窗口里出现过唤醒”即可；必须是携带已授权自动任务 command
            # 的同一 WakeEvent，才能证明任务没有绕过 Agent 会话门控。
            "wake_accepted": matched_wake is not None,
            "automatic_mission_asr_final": automatic_final is not None,
            "strict_core_schema_v4_passed": _strict_core_schema_v4_passed(
                core_report
            ),
        }
        # 原始快照保留在 envelope 中，而不是只输出聚合布尔值；失败时才能区分
        # 麦克风无输入、VAD 未闭合、唤醒拒绝与 ASR 意图不匹配。
        return {
            "schema_version": 1,
            "evidence_kind": VOICE_EVIDENCE_KIND,
            "session_id": str(core_report.get("session_id", "")),
            "agent_mode": agent_mode,
            "trigger_source": "live_voice",
            "passed": all(checks.values()),
            "checks": checks,
            "voice_window": {
                "started_at_ns": self._window_started_at_ns,
                "audio_metrics": [asdict(item) for item in self._audio],
                "endpoint_events": [asdict(item) for item in self._endpoints],
                "wake_events": [asdict(item) for item in self._wake_events],
                "asr_finals": [asdict(item) for item in self._asr_finals],
                "matched_endpoint": (
                    {
                        "speech_started_at_ns": endpoint[0],
                        "speech_ended_at_ns": endpoint[1],
                    }
                    if endpoint
                    else None
                ),
                "matched_automatic_mission_final": (
                    asdict(automatic_final) if automatic_final else None
                ),
                "matched_wake_event": (
                    asdict(matched_wake) if matched_wake else None
                ),
            },
            "core_report": deepcopy(dict(core_report)),
        }
