import math

from embodied_agent_interfaces.msg import AgentTurnMetrics
from embodied_agent_core.metrics_transport import (
    agent_turn_metrics_message_to_dict,
    agent_turn_metrics_to_message,
)


def test_online_metrics_round_trip_preserves_common_latency_and_target_status():
    message = agent_turn_metrics_to_message(
        "online",
        {
            "llm_first_token_ms": 210.5,
            "asr_to_first_token_ms": 350.0,
            "tts_first_audio_ms": 95.0,
            "llm_target_met": True,
            "tts_target_met": True,
        },
    )

    assert message.source == "online"
    assert message.llm_target_status == AgentTurnMetrics.TARGET_MET
    assert math.isnan(message.asr_finalize_ms)
    report = agent_turn_metrics_message_to_dict(message)
    assert report["llm_first_token_ms"] == 210.5
    assert report["asr_finalize_ms"] is None
    assert report["llm_target_met"] is True
    assert report["e2e_target_met"] is None


def test_offline_metrics_round_trip_preserves_llama_and_double_buffer_evidence():
    message = agent_turn_metrics_to_message(
        "offline",
        {
            "asr_finalize_ms": 410.0,
            "llm_first_token_ms": 520.0,
            "end_to_first_audio_ms": 1350.0,
            "turn_complete_ms": 2200.0,
            "asr_target_met": True,
            "e2e_target_met": True,
            "llm_provider": {
                "model": "qwen-q8",
                "request_count": 2,
                "retry_count": 1,
                "token_count": 12,
                "stream_chunks": 10,
                "prompt_tokens": 30,
                "completion_tokens": 12,
                "first_token_ms": 510.0,
                "total_ms": 1600.0,
                "tokens_per_s": 7.5,
                "decode_ms": 1090.0,
                "decode_tokens_per_s": 10.1,
                "first_token_target_met": True,
            },
            "tts_pipeline": {
                "text_chunks": 2,
                "synth_calls": 2,
                "audio_chunks": 9,
                "synth_total_ms": 280.0,
                "first_text_to_first_audio_ms": 145.0,
                "message_buffer_dropped": 0,
                "audio_buffer_dropped": 1,
                "message_buffer_high_watermark": 2,
                "audio_buffer_high_watermark": 2,
            },
        },
    )

    report = agent_turn_metrics_message_to_dict(message)
    assert report["source"] == "offline"
    assert report["llm_provider"]["model"] == "qwen-q8"
    assert report["llm_provider"]["decode_tokens_per_s"] == 10.1
    assert report["tts_pipeline"]["audio_chunks"] == 9
    assert report["tts_pipeline"]["audio_buffer_dropped"] == 1
    assert report["e2e_target_met"] is True


def test_invalid_optional_values_become_explicit_unknowns_instead_of_zero_latency():
    message = agent_turn_metrics_to_message(
        "offline",
        {
            "llm_first_token_ms": None,
            "end_to_first_audio_ms": "not-a-number",
            "llm_provider": {"prompt_tokens": None},
        },
    )

    assert math.isnan(message.llm_first_token_ms)
    assert message.llm_prompt_tokens == 0
    assert message.e2e_target_status == AgentTurnMetrics.TARGET_UNKNOWN
