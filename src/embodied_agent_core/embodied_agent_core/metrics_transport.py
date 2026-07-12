"""Agent turn 指标与 ROS 2 强类型消息之间的唯一转换边界。"""

import math
from typing import Mapping

from embodied_agent_interfaces.msg import AgentTurnMetrics


_FLOAT_FIELDS = (
    "asr_finalize_ms",
    "asr_to_first_token_ms",
    "llm_first_token_ms",
    "tts_first_audio_ms",
    "end_to_first_audio_ms",
    "turn_complete_ms",
)


def _finite_or_nan(value) -> float:
    if value is None:
        return math.nan
    try:
        number = float(value)
    except (TypeError, ValueError):
        return math.nan
    return number if math.isfinite(number) else math.nan


def _optional_float(value: float) -> float | None:
    return float(value) if math.isfinite(value) else None


def _count(value) -> int:
    try:
        return max(0, int(value or 0))
    except (TypeError, ValueError):
        return 0


def _target_status(value) -> int:
    if value is True:
        return AgentTurnMetrics.TARGET_MET
    if value is False:
        return AgentTurnMetrics.TARGET_MISSED
    return AgentTurnMetrics.TARGET_UNKNOWN


def _target_value(status: int) -> bool | None:
    if status == AgentTurnMetrics.TARGET_MET:
        return True
    if status == AgentTurnMetrics.TARGET_MISSED:
        return False
    return None


def agent_turn_metrics_to_message(
    source: str,
    report: Mapping,
    *,
    stamp=None,
) -> AgentTurnMetrics:
    """把在线/离线内部报告规范化为同一个 ROS schema。"""

    message = AgentTurnMetrics()
    if stamp is not None:
        message.stamp = stamp
    message.source = str(source)
    for field in _FLOAT_FIELDS:
        setattr(message, field, _finite_or_nan(report.get(field)))

    message.asr_target_status = _target_status(report.get("asr_target_met"))
    message.llm_target_status = _target_status(report.get("llm_target_met"))
    message.tts_target_status = _target_status(report.get("tts_target_met"))
    message.e2e_target_status = _target_status(report.get("e2e_target_met"))

    llm = report.get("llm_provider") or {}
    message.llm_model = str(llm.get("model") or "")
    for target, source_key in (
        ("llm_request_count", "request_count"),
        ("llm_retry_count", "retry_count"),
        ("llm_token_count", "token_count"),
        ("llm_stream_chunks", "stream_chunks"),
        ("llm_prompt_tokens", "prompt_tokens"),
        ("llm_completion_tokens", "completion_tokens"),
    ):
        setattr(message, target, _count(llm.get(source_key)))
    for target, source_key in (
        ("llm_provider_first_token_ms", "first_token_ms"),
        ("llm_total_ms", "total_ms"),
        ("llm_tokens_per_s", "tokens_per_s"),
        ("llm_decode_ms", "decode_ms"),
        ("llm_decode_tokens_per_s", "decode_tokens_per_s"),
    ):
        setattr(message, target, _finite_or_nan(llm.get(source_key)))
    message.llm_provider_first_token_target_status = _target_status(
        llm.get("first_token_target_met")
    )
    message.llm_error = str(llm.get("error") or "")

    tts = report.get("tts_pipeline") or {}
    for target in ("tts_text_chunks", "tts_synth_calls", "tts_audio_chunks"):
        setattr(message, target, _count(tts.get(target.removeprefix("tts_"))))
    message.tts_synth_total_ms = _finite_or_nan(tts.get("synth_total_ms"))
    message.tts_first_text_to_first_audio_ms = _finite_or_nan(
        tts.get("first_text_to_first_audio_ms")
    )
    for target in (
        "message_buffer_dropped",
        "audio_buffer_dropped",
        "message_buffer_high_watermark",
        "audio_buffer_high_watermark",
    ):
        # 顶层保留 dropped 计数是 OfflineLatency 的历史领域结构；
        # typed wire 只保留一份明确字段，优先使用更完整的 TTS pipeline 统计。
        setattr(message, target, _count(tts.get(target, report.get(target))))
    return message


def agent_turn_metrics_message_to_dict(message: AgentTurnMetrics) -> dict:
    """为日志、JSON 报告和现有统计函数恢复稳定的 Python 报告结构。"""

    report = {
        field: _optional_float(getattr(message, field)) for field in _FLOAT_FIELDS
    }
    report.update(
        {
            "source": message.source,
            "asr_target_met": _target_value(message.asr_target_status),
            "llm_target_met": _target_value(message.llm_target_status),
            "tts_target_met": _target_value(message.tts_target_status),
            "e2e_target_met": _target_value(message.e2e_target_status),
            "message_buffer_dropped": int(message.message_buffer_dropped),
            "audio_buffer_dropped": int(message.audio_buffer_dropped),
            "llm_provider": {
                "model": message.llm_model,
                "request_count": int(message.llm_request_count),
                "retry_count": int(message.llm_retry_count),
                "token_count": int(message.llm_token_count),
                "stream_chunks": int(message.llm_stream_chunks),
                "prompt_tokens": int(message.llm_prompt_tokens),
                "completion_tokens": int(message.llm_completion_tokens),
                "first_token_ms": _optional_float(
                    message.llm_provider_first_token_ms
                ),
                "total_ms": _optional_float(message.llm_total_ms),
                "tokens_per_s": _optional_float(message.llm_tokens_per_s),
                "decode_ms": _optional_float(message.llm_decode_ms),
                "decode_tokens_per_s": _optional_float(
                    message.llm_decode_tokens_per_s
                ),
                "first_token_target_met": _target_value(
                    message.llm_provider_first_token_target_status
                ),
                "error": message.llm_error or None,
            },
            "tts_pipeline": {
                "text_chunks": int(message.tts_text_chunks),
                "synth_calls": int(message.tts_synth_calls),
                "audio_chunks": int(message.tts_audio_chunks),
                "synth_total_ms": _optional_float(message.tts_synth_total_ms),
                "first_text_to_first_audio_ms": _optional_float(
                    message.tts_first_text_to_first_audio_ms
                ),
                "message_buffer_dropped": int(message.message_buffer_dropped),
                "audio_buffer_dropped": int(message.audio_buffer_dropped),
                "message_buffer_high_watermark": int(
                    message.message_buffer_high_watermark
                ),
                "audio_buffer_high_watermark": int(
                    message.audio_buffer_high_watermark
                ),
            },
        }
    )
    return report
