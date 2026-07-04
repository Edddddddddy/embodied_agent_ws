#!/usr/bin/env python3
"""Human-facing monitor for long-running voice control demos."""

import argparse
import importlib.util
import json
import signal
import sys
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _load_audio_calibration_module():
    """Load sibling calibration helpers when monitor is imported in tests.

    直接运行脚本时，Python 会把 scripts/ 放进 sys.path，普通 import 能工作；
    测试通过 spec_from_file_location 导入时不一定有这个路径，因此这里显式兜底。
    """

    try:
        import audio_frontend_calibration

        return audio_frontend_calibration
    except ModuleNotFoundError:
        module_path = Path(__file__).with_name("audio_frontend_calibration.py")
        spec = importlib.util.spec_from_file_location(
            "audio_frontend_calibration", module_path
        )
        if spec is None or spec.loader is None:
            raise
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        return module


audio_calibration = _load_audio_calibration_module()


def _json_dict(serialized: str) -> dict[str, Any]:
    try:
        payload = json.loads(serialized)
    except json.JSONDecodeError:
        return {}
    return payload if isinstance(payload, dict) else {}


def format_session_state(state: str) -> str:
    return f"[session] {state}"


def format_wake_event(serialized: str) -> str:
    payload = _json_dict(serialized)
    provider = payload.get("provider", "unknown")
    kind = payload.get("kind", "unknown")
    return f"[wake] {provider}:{kind}"


def format_kws_event(serialized: str) -> str:
    payload = _json_dict(serialized)
    provider = payload.get("provider", "unknown")
    transcript = payload.get("transcript", "")
    score = payload.get("score", "?")
    return f"[kws] {provider} detected {transcript} score={score}"


def format_kws_score(serialized: str) -> str:
    payload = _json_dict(serialized)
    provider = payload.get("provider", "unknown")
    keyword = payload.get("top_keyword", "unknown")
    score = float(payload.get("top_score", 0.0))
    threshold = float(payload.get("threshold", 0.0))
    above = payload.get("above_threshold", False)
    return (
        f"[kws-score] {provider} {keyword}={score:.3f} "
        f"threshold={threshold:.3f} above={above}"
    )


def format_audio_metrics(serialized: str) -> str:
    payload = _json_dict(serialized)
    rms = payload.get("rms", 0.0)
    peak = payload.get("peak", 0)
    speech = payload.get("speech", False)
    dropped = payload.get("dropped_input_frames", 0)
    enhancer = payload.get("audio_enhancer_active", "unknown")
    aec = payload.get("aec_active", False)
    ns = payload.get("noise_suppression_active", False)
    agc = payload.get("auto_gain_active", False)
    return (
        f"[audio] rms={float(rms):.4f} peak={peak} speech={speech} dropped={dropped} "
        f"enhancer={enhancer} aec={aec} ns={ns} agc={agc}"
    )


def format_asr_final(text: str) -> str:
    return f"[asr] {text}"


def format_queue_state(state: str, size: int | None = None) -> str:
    if state == "queued":
        suffix = "" if size is None else f" size={size}"
        return f"[queue] enqueue{suffix}"
    if state == "queue_full":
        return "[queue] full"
    return f"[state] {state}"


def format_queue_event(serialized: str) -> str:
    payload = _json_dict(serialized)
    event = payload.get("event", "unknown")
    text = payload.get("text") or ""
    size = payload.get("size")
    dropped = payload.get("dropped", 0)
    reason = payload.get("reason") or ""
    if event == "enqueue":
        return f"[queue] enqueue {text} size={size}"
    if event == "rejected":
        return f"[queue] rejected {text} reason={reason}"
    if event == "clear":
        return f"[queue] clear dropped={dropped}"
    if event == "expired":
        return f"[queue] expired {text} reason={reason} size={size}"
    return f"[queue] {event}"


def format_execution_event(serialized: str) -> str:
    payload = _json_dict(serialized)
    event = payload.get("event", "unknown")
    text = payload.get("text") or ""
    if event == "finished":
        reason = payload.get("reason") or ""
        return f"[exec] finished {text} {reason}".rstrip()
    return f"[exec] {event} {text}".rstrip()


def format_action_candidate(serialized: str) -> str:
    payload = _json_dict(serialized)
    name = payload.get("name", "unknown")
    return f"[action] executing {name}"


def format_action_feedback(serialized: str) -> str:
    payload = _json_dict(serialized)
    phase_names = {
        1: "accepted",
        2: "executing",
        3: "stopping",
    }
    phase = phase_names.get(payload.get("phase"), str(payload.get("phase", "unknown")))
    progress = max(0.0, min(1.0, float(payload.get("progress", 0.0))))
    detail = payload.get("detail") or ""
    return f"[feedback] {phase} {progress * 100:.0f}% {detail}".rstrip()


def format_action_result(serialized: str) -> str:
    payload = _json_dict(serialized)
    if payload.get("success") is True:
        return f"[result] {payload.get('message', 'succeeded')}"
    if "status" in payload:
        return f"[ack] {payload.get('action', 'unknown')} {payload.get('status')}"
    return f"[result] {payload.get('message', 'unknown')}"


def format_recognition_feedback(serialized: str) -> str:
    payload = _json_dict(serialized)
    if payload.get("status") == "normalized":
        original = payload.get("original", "")
        normalized = payload.get("normalized", "")
        return f"[normalize] {original} -> {normalized}"
    if payload.get("status") == "ignored":
        reason = payload.get("reason", "unknown")
        transcript = payload.get("transcript", "")
        return f"[ignore] {reason} {transcript}".rstrip()
    if payload.get("status") == "retry":
        attempt = payload.get("attempt", "?")
        prompt = payload.get("prompt", "请再说一次")
        return f"[retry] attempt={attempt} {prompt}"
    if payload.get("status") == "queue_rejected":
        reason = payload.get("reason", "unknown")
        transcript = payload.get("transcript", "")
        size = payload.get("queue_size", "?")
        return f"[queue-feedback] {reason} {transcript} size={size}".rstrip()
    if payload.get("status") == "session_timeout":
        prompt = payload.get("prompt", "会话已超时，请重新唤醒")
        transcript = payload.get("transcript", "")
        return f"[session-timeout] {prompt} {transcript}".rstrip()
    return "[feedback] " + (payload.get("reason") or "unknown")


def _interrupt_monitor(_signum, _frame) -> None:
    raise KeyboardInterrupt


def install_signal_handlers() -> None:
    signal.signal(signal.SIGINT, _interrupt_monitor)
    signal.signal(signal.SIGTERM, _interrupt_monitor)


@dataclass
class MonitorStats:
    """长时间语音演示的轻量统计器。

    统计只依赖已有 JSON topic，退出 monitor 时输出一行 summary，帮助判断问题是在
    ASR、过滤、队列还是执行阶段。
    """

    asr: int = 0
    wake: int = 0
    sleep: int = 0
    retry: int = 0
    timeout: int = 0
    ignored: int = 0
    normalized: int = 0
    enqueued: int = 0
    rejected: int = 0
    expired: int = 0
    started: int = 0
    finished: int = 0
    succeeded: int = 0
    failed: int = 0
    audio_sample_limit: int = 600
    audio_samples: deque[Any] = field(init=False)

    def __post_init__(self) -> None:
        # /audio/frontend_metrics 默认约 0.5s 一条；600 条约覆盖最近 5 分钟。
        # 长时间演示只需要近期音频健康趋势，避免 monitor 常驻时无限增长。
        self.audio_samples = deque(maxlen=max(1, int(self.audio_sample_limit)))

    def record_wake(self, serialized: str) -> None:
        kind = _json_dict(serialized).get("kind")
        if kind == "wake":
            self.wake += 1
        elif kind == "sleep":
            self.sleep += 1

    def record_asr(self, _text: str) -> None:
        self.asr += 1

    def record_recognition_feedback(self, serialized: str) -> None:
        payload = _json_dict(serialized)
        status = payload.get("status")
        if status == "ignored":
            self.ignored += 1
        elif status == "normalized":
            self.normalized += 1
        elif status == "retry":
            self.retry += 1
        elif status == "session_timeout":
            self.timeout += 1

    def record_queue(self, serialized: str) -> None:
        event = _json_dict(serialized).get("event")
        if event == "enqueue":
            self.enqueued += 1
        elif event == "rejected":
            self.rejected += 1
        elif event == "expired":
            self.expired += 1

    def record_execution(self, serialized: str) -> None:
        event = _json_dict(serialized).get("event")
        if event == "started":
            self.started += 1
        elif event == "finished":
            self.finished += 1

    def record_result(self, serialized: str) -> None:
        payload = _json_dict(serialized)
        if "success" not in payload:
            return
        if payload.get("success") is True:
            self.succeeded += 1
        else:
            self.failed += 1

    def record_audio(self, serialized: str) -> None:
        sample = audio_calibration.parse_audio_metrics(serialized)
        if sample is not None:
            self.audio_samples.append(sample)

    def format_summary(self) -> str:
        audio_report = None
        base = (
            f"[summary] wake={self.wake} sleep={self.sleep} retry={self.retry} "
            f"timeout={self.timeout} "
            f"asr={self.asr} ignored={self.ignored} "
            f"normalized={self.normalized} enqueued={self.enqueued} "
            f"rejected={self.rejected} expired={self.expired} started={self.started} "
            f"finished={self.finished} succeeded={self.succeeded} failed={self.failed}"
        )
        parts = [base]
        if not self.audio_samples:
            parts.extend(self.format_advice())
            return "\n".join(parts)
        audio_report = audio_calibration.analyze_audio_health(self.audio_samples)
        warnings = ",".join(audio_report.warnings) if audio_report.warnings else "none"
        parts.append(
            f"[summary-audio] samples={audio_report.sample_count} "
            f"profile={audio_report.recommended_voice_profile} "
            f"reason={audio_report.profile_reason} "
            f"mean_rms={audio_report.mean_rms:.4f} max_rms={audio_report.max_rms:.4f} "
            f"speech_ratio={audio_report.speech_ratio:.2f} "
            f"dropped_input_delta={audio_report.dropped_input_delta} "
            f"warnings={warnings}"
        )
        parts.extend(self.format_advice(audio_report))
        return "\n".join(parts)

    def format_advice(self, audio_report=None) -> list[str]:
        """根据长时间演示统计给出下一步排障建议。"""

        advice: list[str] = []
        if self.asr == 0:
            advice.append(
                "[advice] 没有收到 ASR final：检查麦克风 source、VAD 阈值，或先运行 audio_frontend_calibration.py。"
            )
        if self.ignored >= 3 and self.ignored >= max(3, self.asr // 2):
            advice.append(
                "[advice] ignored 偏高：可能是 ASR 抖动或语气词过多，可调整 CONTINUOUS_DUPLICATE_WINDOW_S 或检查识别文本。"
            )
        if self.rejected > 0:
            advice.append(
                "[advice] 出现 queue_full：请放慢连续说话节奏，或适当增大 CONTINUOUS_COMMAND_QUEUE_SIZE。"
            )
        if self.expired > 0:
            advice.append(
                "[advice] 有命令过期：机器人执行较慢或说话过快，可缩短演示话术或增大 CONTINUOUS_COMMAND_MAX_AGE。"
            )
        if self.failed > 0:
            advice.append(
                "[advice] 有动作失败：查看 [result]/[feedback] 与仿真安全状态，必要时先说“停下”。"
            )
        if audio_report is not None:
            warning_set = set(audio_report.warnings)
            if "vad_threshold_may_be_too_low_or_environment_noisy" in warning_set:
                advice.append(
                    "[advice] 环境可能持续触发 speech：建议 VOICE_CONTROL_PROFILE=noisy_room，或提高 SPEECH_START_THRESHOLD。"
                )
            if "vad_threshold_may_be_too_high" in warning_set:
                advice.append(
                    "[advice] VAD 可能太保守：建议 VOICE_CONTROL_PROFILE=quiet，或降低 SPEECH_START_THRESHOLD。"
                )
            if "audio_capture_overrun" in warning_set:
                advice.append(
                    "[advice] 麦克风输入有丢帧：关闭重负载程序，或降低 Gazebo/GUI 负载后重试。"
                )
        return advice


class ContinuousVoiceMonitor(Node):
    def __init__(self, *, audio_sample_limit: int = 600):
        super().__init__("continuous_voice_monitor")
        self._queued_seen = 0
        self._structured_queue_seen = False
        self._stats = MonitorStats(audio_sample_limit=audio_sample_limit)
        self.create_subscription(String, "/agent/session_state", self._on_session, 10)
        self.create_subscription(String, "/agent/wake_event", self._on_wake, 10)
        self.create_subscription(String, "/agent/kws_event", self._on_kws, 10)
        self.create_subscription(String, "/agent/kws_score", self._on_kws_score, 10)
        self.create_subscription(String, "/audio/frontend_metrics", self._on_audio, 10)
        self.create_subscription(String, "/agent/asr_final", self._on_asr, 10)
        self.create_subscription(String, "/agent/state", self._on_state, 10)
        self.create_subscription(String, "/agent/command_queue", self._on_queue, 10)
        self.create_subscription(
            String, "/agent/command_execution", self._on_execution, 10
        )
        self.create_subscription(String, "/agent/action_candidate", self._on_action, 10)
        self.create_subscription(
            String, "/robot/action_feedback", self._on_action_feedback, 10
        )
        self.create_subscription(
            String, "/agent/recognition_feedback", self._on_feedback, 10
        )
        self.create_subscription(String, "/robot/action_result", self._on_result, 10)
        self.create_subscription(String, "/robot/action_ack", self._on_result, 10)

    def _emit(self, line: str) -> None:
        print(line, flush=True)

    def _on_session(self, message: String) -> None:
        self._emit(format_session_state(message.data))

    def _on_wake(self, message: String) -> None:
        self._stats.record_wake(message.data)
        self._emit(format_wake_event(message.data))

    def _on_kws(self, message: String) -> None:
        self._emit(format_kws_event(message.data))

    def _on_kws_score(self, message: String) -> None:
        self._emit(format_kws_score(message.data))

    def _on_audio(self, message: String) -> None:
        self._stats.record_audio(message.data)
        self._emit(format_audio_metrics(message.data))

    def _on_asr(self, message: String) -> None:
        self._stats.record_asr(message.data)
        self._emit(format_asr_final(message.data))

    def _on_state(self, message: String) -> None:
        if message.data == "queued" and not self._structured_queue_seen:
            self._queued_seen += 1
            self._emit(format_queue_state(message.data, self._queued_seen))
        elif message.data == "queue_full":
            self._emit(format_queue_state(message.data))

    def _on_queue(self, message: String) -> None:
        self._structured_queue_seen = True
        self._stats.record_queue(message.data)
        self._emit(format_queue_event(message.data))

    def _on_execution(self, message: String) -> None:
        self._stats.record_execution(message.data)
        self._emit(format_execution_event(message.data))

    def _on_action(self, message: String) -> None:
        self._emit(format_action_candidate(message.data))

    def _on_action_feedback(self, message: String) -> None:
        self._emit(format_action_feedback(message.data))

    def _on_feedback(self, message: String) -> None:
        self._stats.record_recognition_feedback(message.data)
        self._emit(format_recognition_feedback(message.data))

    def _on_result(self, message: String) -> None:
        self._stats.record_result(message.data)
        self._emit(format_action_result(message.data))

    def emit_summary(self) -> None:
        self._emit(self._stats.format_summary())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--audio-sample-limit",
        type=int,
        default=600,
        help="保留最近 N 条 /audio/frontend_metrics 用于退出 summary，避免长时间演示无限增长。",
    )
    args = parser.parse_args()
    install_signal_handlers()
    rclpy.init()
    node = ContinuousVoiceMonitor(audio_sample_limit=args.audio_sample_limit)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.emit_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
