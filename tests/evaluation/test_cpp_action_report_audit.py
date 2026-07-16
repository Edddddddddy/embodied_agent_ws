from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "audit_cpp_action_reports", ROOT / "scripts" / "audit_cpp_action_reports.py"
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _report(outcome: str, status: int) -> dict[str, object]:
    return {
        "observed_outcome": outcome,
        "expected_outcome": outcome,
        "passed": True,
        "goal_accepted": True,
        "action_status": status,
        "feedback_count": 2,
        "cancel_requested": outcome == "canceled",
        "transport_code": "ABORTED" if outcome == "timed_out" else "SUCCEEDED",
    }


def test_accepts_complete_success_cancel_timeout_contract() -> None:
    result = MODULE.audit_reports(
        [_report("succeeded", 1), _report("canceled", 3), _report("timed_out", 4)],
        wire_feedback_count=3,
    )
    assert result["passed"] is True
    assert result["failures"] == []


def test_rejects_missing_feedback_and_cancel_request() -> None:
    canceled = _report("canceled", 3)
    canceled["feedback_count"] = 0
    canceled["cancel_requested"] = False
    reports = [_report("succeeded", 1), canceled, _report("timed_out", 4)]
    for report in reports:
        report["feedback_count"] = 0
    result = MODULE.audit_reports(reports, wire_feedback_count=0)
    assert result["passed"] is False
    assert "no ROS Action feedback observed" in result["failures"]
    assert "canceled: client did not issue ROS Action cancel" in result["failures"]
