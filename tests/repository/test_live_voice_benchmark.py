from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "evaluate_live_voice_benchmark", ROOT / "scripts" / "evaluate_live_voice_benchmark.py"
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


SCENARIO = {
    "name": "test",
    "commands": [
        {"text": f"命令{index}", "action": "move" if index % 2 == 0 else "turn"}
        for index in range(10)
    ],
}


def _report(duration: float = 180.0) -> dict:
    return {
        "duration_s": duration,
        "capture_source": "real_microphone",
        "asr_samples": [f"命令{index}" for index in range(10)],
        "action_candidate_samples": [
            {"name": "move" if index % 2 == 0 else "turn"} for index in range(10)
        ],
        "action_success_count": 10,
        "successful_action_samples": [
            {
                "success": True,
                "action_name": "move" if index % 2 == 0 else "turn",
            }
            for index in range(10)
        ],
        "saw_awake": True,
        "saw_sleeping": True,
        "final_cmd_vel_zero": True,
        "metrics_samples": [
            {
                "asr_to_first_token_ms": 500.0,
                "llm_first_token_ms": 300.0,
                "tts_first_audio_ms": 200.0,
            }
        ],
        "action_e2e_latency_ms": [1200.0, 1800.0],
    }


def test_benchmark_passes_complete_three_minute_evidence() -> None:
    result = MODULE.evaluate(_report(), SCENARIO)
    assert result["passed"] is True
    assert result["recognition_rate"] == 1.0
    assert result["action_accuracy"] == 1.0
    assert result["latency"]["asr_to_first_token_ms"]["p95_ms"] == 500.0
    assert result["latency"]["asr_final_to_action_result_ms"]["p95_ms"] == 1800.0
    assert result["evidence_scope"] == "operator_declared_real_microphone"


def test_benchmark_marks_short_run_and_false_trigger_as_incomplete() -> None:
    report = _report(duration=60.0)
    report["action_candidate_samples"].extend({"name": "wave"} for _ in range(3))
    result = MODULE.evaluate(report, SCENARIO)
    assert result["passed"] is False
    assert result["checks"]["duration_at_least_180s"] is False
    assert result["checks"]["false_trigger_rate_at_most_10pct"] is False
    assert result["evidence_scope"] == "synthetic_short_or_unspecified"


def test_benchmark_reads_offline_latency_aliases_without_inventing_values() -> None:
    report = _report()
    report["metrics_samples"] = [
        {
            "asr_finalize_ms": 420.0,
            "llm_first_token_ms": 310.0,
            "end_to_first_audio_ms": 1450.0,
            "turn_complete_ms": 2200.0,
        }
    ]
    result = MODULE.evaluate(report, SCENARIO)
    assert result["latency"]["asr_finalize_ms"]["median_ms"] == 420.0
    assert result["latency"]["tts_first_audio_ms"]["median_ms"] == 1450.0
    assert result["latency"]["asr_to_first_token_ms"]["count"] == 0


def test_benchmark_does_not_credit_unassociated_success_totals() -> None:
    report = _report()
    report["successful_action_samples"] = []
    report["action_success_count"] = 99
    result = MODULE.evaluate(report, SCENARIO)
    assert result["action_success_rate"] == 0.0
    assert result["checks"]["action_success_rate_at_least_80pct"] is False


def test_action_alignment_does_not_cascade_after_one_missing_repeated_action() -> None:
    """真实报告漏掉左转后，后续正确 move/turn 不应被贪心游标连带错扣。"""
    expected = [
        "move", "turn", "move", "turn", "arc",
        "wave", "set_led", "navigate_to", "cancel_navigation", "stop",
    ]
    observed = ["move", "move", "turn", "arc", "wave", "navigate_to", "stop"]

    assert MODULE._monotonic_action_match_count(expected, observed) == 7


def test_offline_hotwords_cover_all_live_benchmark_domains() -> None:
    hotwords = (
        ROOT / "src" / "embodied_offline_agent" / "config" / "hotwords_zh.txt"
    ).read_text(encoding="utf-8").splitlines()

    required = {
        "挥手两次", "把灯设成蓝色", "蓝色", "去门口", "取消导航",
    }
    assert required.issubset(set(hotwords))
