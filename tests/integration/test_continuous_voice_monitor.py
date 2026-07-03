#!/usr/bin/env python3
"""Unit-style checks for the human-facing continuous voice monitor."""

import importlib.util
import json
import sys
import types
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MONITOR = ROOT / "scripts" / "continuous_voice_monitor.py"
sys.path.insert(0, str(ROOT / "scripts"))

sys.modules.setdefault("rclpy", types.SimpleNamespace())
sys.modules.setdefault(
    "rclpy.node",
    types.SimpleNamespace(Node=object),
)
sys.modules.setdefault(
    "std_msgs.msg",
    types.SimpleNamespace(String=object),
)

spec = importlib.util.spec_from_file_location("continuous_voice_monitor", MONITOR)
monitor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor)


def test_monitor_formats_session_and_wake_events():
    wake = json.dumps({"kind": "wake", "provider": "text"}, ensure_ascii=False)
    kws = json.dumps(
        {"provider": "mock_kws", "transcript": "小智", "score": 1.0},
        ensure_ascii=False,
    )
    audio = json.dumps(
        {
            "rms": 0.0305,
            "peak": 1000,
            "speech": True,
            "dropped_input_frames": 0,
            "audio_enhancer_active": "nlms",
            "aec_active": True,
            "noise_suppression_active": False,
            "auto_gain_active": False,
        },
        ensure_ascii=False,
    )

    assert monitor.format_session_state("awake") == "[session] awake"
    assert monitor.format_wake_event(wake) == "[wake] text:wake"
    assert monitor.format_kws_event(kws) == "[kws] mock_kws detected 小智 score=1.0"
    assert (
        monitor.format_audio_metrics(audio)
        == "[audio] rms=0.0305 peak=1000 speech=True dropped=0 enhancer=nlms aec=True ns=False agc=False"
    )


def test_monitor_formats_kws_score_events():
    score = json.dumps(
        {
            "provider": "openwakeword_test",
            "top_keyword": "fake_wake",
            "top_score": 0.91,
            "threshold": 0.5,
            "above_threshold": True,
        },
        ensure_ascii=False,
    )

    assert (
        monitor.format_kws_score(score)
        == "[kws-score] openwakeword_test fake_wake=0.910 threshold=0.500 above=True"
    )


def test_monitor_formats_asr_queue_action_and_result_events():
    queue_event = json.dumps(
        {"event": "enqueue", "text": "向前走一秒", "size": 2},
        ensure_ascii=False,
    )
    expired_event = json.dumps(
        {
            "event": "expired",
            "text": "向前走一秒",
            "size": 1,
            "reason": "stale_command",
        },
        ensure_ascii=False,
    )
    execution_event = json.dumps(
        {"event": "started", "text": "向前走一秒"},
        ensure_ascii=False,
    )
    candidate = json.dumps(
        {"name": "move", "arguments": {"linear_x": 0.2, "duration_s": 1.0}},
        ensure_ascii=False,
    )
    result = json.dumps({"success": True, "message": "succeeded"}, ensure_ascii=False)
    feedback = json.dumps(
        {"phase": 2, "progress": 0.42, "detail": "mock_execution"},
        ensure_ascii=False,
    )

    assert monitor.format_asr_final("向前走一秒") == "[asr] 向前走一秒"
    assert monitor.format_queue_event(queue_event) == "[queue] enqueue 向前走一秒 size=2"
    assert (
        monitor.format_queue_event(expired_event)
        == "[queue] expired 向前走一秒 reason=stale_command size=1"
    )
    assert monitor.format_execution_event(execution_event) == "[exec] started 向前走一秒"
    assert monitor.format_action_candidate(candidate) == "[action] executing move"
    assert monitor.format_action_feedback(feedback) == "[feedback] executing 42% mock_execution"
    assert monitor.format_action_result(result) == "[result] succeeded"


def test_monitor_formats_normalization_feedback():
    feedback = json.dumps(
        {
            "status": "normalized",
            "reason": "command_normalized",
            "original": "钱进一秒",
            "normalized": "前进一秒",
        },
        ensure_ascii=False,
    )

    assert monitor.format_recognition_feedback(feedback) == "[normalize] 钱进一秒 -> 前进一秒"


def test_monitor_formats_ignored_recognition_feedback():
    feedback = json.dumps(
        {
            "status": "ignored",
            "reason": "filler",
            "transcript": "嗯。",
        },
        ensure_ascii=False,
    )

    assert monitor.format_recognition_feedback(feedback) == "[ignore] filler 嗯。"


def test_monitor_stats_summarizes_long_running_session():
    stats = monitor.MonitorStats()
    stats.record_wake(json.dumps({"kind": "wake", "provider": "text"}, ensure_ascii=False))
    stats.record_wake(json.dumps({"kind": "sleep", "provider": "text"}, ensure_ascii=False))
    stats.record_asr("向前走一秒")
    stats.record_recognition_feedback(
        json.dumps({"status": "retry", "attempt": 1}, ensure_ascii=False)
    )
    stats.record_recognition_feedback(
        json.dumps({"status": "ignored", "reason": "filler"}, ensure_ascii=False)
    )
    stats.record_recognition_feedback(
        json.dumps({"status": "normalized", "reason": "command_normalized"}, ensure_ascii=False)
    )
    stats.record_queue(
        json.dumps({"event": "enqueue", "text": "向前走一秒", "size": 1}, ensure_ascii=False)
    )
    stats.record_queue(
        json.dumps({"event": "expired", "text": "左转九十度", "size": 0}, ensure_ascii=False)
    )
    stats.record_execution(
        json.dumps({"event": "started", "text": "向前走一秒"}, ensure_ascii=False)
    )
    stats.record_execution(
        json.dumps(
            {
                "event": "finished",
                "text": "向前走一秒",
                "success": True,
                "reason": "completed",
            },
            ensure_ascii=False,
        )
    )
    stats.record_result(json.dumps({"success": True, "message": "succeeded"}))

    assert stats.format_summary() == (
        "[summary] wake=1 sleep=1 retry=1 asr=1 ignored=1 normalized=1 enqueued=1 expired=1 "
        "started=1 finished=1 succeeded=1 failed=0"
    )


def test_monitor_stats_summarizes_audio_health_and_recommended_profile():
    stats = monitor.MonitorStats()
    for _ in range(4):
        stats.record_audio(
            json.dumps(
                {
                    "rms": 0.02,
                    "peak": 900,
                    "speech": True,
                    "dropped_input_frames": 0,
                    "dropped_playback_chunks": 0,
                    "vad_provider": "energy",
                    "audio_enhancer_requested": "nlms",
                    "audio_enhancer_active": "nlms",
                    "aec_active": True,
                    "noise_suppression_active": False,
                    "auto_gain_active": False,
                },
                ensure_ascii=False,
            )
        )

    summary = stats.format_summary()

    assert "[summary-audio] samples=4" in summary
    assert "profile=noisy_room" in summary
    assert "speech_ratio=1.00" in summary
    assert "mean_rms=0.0200" in summary
    assert "warnings=vad_threshold_may_be_too_low_or_environment_noisy" in summary


def test_monitor_stats_keeps_bounded_recent_audio_samples():
    stats = monitor.MonitorStats(audio_sample_limit=2)
    for rms in (0.005, 0.02, 0.03):
        stats.record_audio(
            json.dumps(
                {
                    "rms": rms,
                    "peak": 900,
                    "speech": True,
                    "dropped_input_frames": 0,
                    "dropped_playback_chunks": 0,
                },
                ensure_ascii=False,
            )
        )

    summary = stats.format_summary()

    assert "[summary-audio] samples=2" in summary
    assert "mean_rms=0.0250" in summary
    assert "max_rms=0.0300" in summary


def test_monitor_exposes_audio_sample_limit_cli_option():
    content = MONITOR.read_text(encoding="utf-8")

    assert "--audio-sample-limit" in content
    assert "ContinuousVoiceMonitor(audio_sample_limit=args.audio_sample_limit)" in content


def test_monitor_signal_handler_uses_keyboard_interrupt_for_summary_path():
    try:
        monitor._interrupt_monitor(None, None)
    except KeyboardInterrupt:
        pass
    else:
        raise AssertionError("monitor signal handler did not enter summary path")
