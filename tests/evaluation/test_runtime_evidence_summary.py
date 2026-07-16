from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "generate_runtime_evidence_summary",
    ROOT / "scripts" / "generate_runtime_evidence_summary.py",
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _live(mode: str, *, passed: bool = True, duration: float = 300.0) -> dict:
    return {
        "passed": passed,
        "source_report_duration_s": duration,
        "agent_mode": mode,
        "capture_source": "real_microphone",
        "evidence_scope": "operator_declared_real_microphone",
        "recognition_rate": 0.9,
        "action_accuracy": 0.9,
        "action_success_rate": 0.9,
        "false_trigger_rate": 0.0,
        "queue_rejected_count": 1,
        "queue_reject_rate": 0.1,
        "checks": {"final_cmd_vel_zero": True},
        "latency": {"turn_complete_ms": {"median_ms": 1200.0, "p95_ms": 1800.0}},
    }


def test_summary_separates_raw_model_from_fallback_and_proves_both_modes():
    summary = MODULE.build_summary(
        online=_live("online"),
        offline=_live("offline"),
        instruction={
            "model_score": 0.375,
            "effective_score": 1.0,
            "total": 8,
            "effective_score_policy": "action_only_after_fallback_and_safety",
        },
        offline_latency={
            "measurement_scope": "speech_endpoint_to_first_tts_pcm_chunk",
            "asr_text": "向前走一秒",
            "metrics": {"turn_complete_ms": 1574.0, "end_to_first_audio_ms": 861.0},
        },
    )

    assert summary["all_long_run_modes_proven"] is True
    instruction = summary["offline_instruction_following"]
    assert instruction["raw_model_action_accuracy"] == 0.375
    assert instruction["fallback_and_safety_effective_accuracy"] == 1.0
    assert instruction["raw_model_target_70pct_met"] is False
    assert summary["offline_turn_latency"]["turn_target_3500ms_met"] is True


def test_summary_marks_missing_or_short_live_evidence_without_faking_pass():
    summary = MODULE.build_summary(
        online=None,
        offline=_live("offline", duration=180.0),
        instruction=None,
        offline_latency=None,
    )

    assert summary["continuous_online_5min"]["status"] == "missing"
    assert summary["continuous_offline_5min"]["status"] == "failed"
    assert summary["all_long_run_modes_proven"] is False
