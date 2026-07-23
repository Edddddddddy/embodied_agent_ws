#!/usr/bin/env python3
"""汇总 C++ typed Action 场景，并校验成功、取消、超时三类终态契约。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


EXPECTED_STATUS = {"succeeded": 1, "canceled": 3, "timed_out": 4}


def audit_reports(
    reports: list[dict[str, Any]], wire_feedback_count: int | None = None
) -> dict[str, Any]:
    failures: list[str] = []
    by_outcome = {str(item.get("observed_outcome")): item for item in reports}

    for outcome, status in EXPECTED_STATUS.items():
        report = by_outcome.get(outcome)
        if report is None:
            failures.append(f"missing scenario: {outcome}")
            continue
        if report.get("expected_outcome") != outcome or not report.get("passed"):
            failures.append(f"{outcome}: expectation did not pass")
        if not report.get("goal_accepted"):
            failures.append(f"{outcome}: goal was not accepted")
        if int(report.get("action_status", -1)) != status:
            failures.append(f"{outcome}: action_status is not {status}")

    client_feedback_count = sum(int(item.get("feedback_count", 0)) for item in reports)
    observed_feedback_count = max(client_feedback_count, wire_feedback_count or 0)
    if observed_feedback_count < 1:
        failures.append("no ROS Action feedback observed")

    canceled = by_outcome.get("canceled", {})
    if canceled and not canceled.get("cancel_requested"):
        failures.append("canceled: client did not issue ROS Action cancel")
    timed_out = by_outcome.get("timed_out", {})
    if timed_out and timed_out.get("transport_code") != "ABORTED":
        failures.append("timed_out: expected ABORTED transport with STATUS_TIMED_OUT")

    return {
        "schema_version": 1,
        "passed": not failures,
        "scenarios": reports,
        "client_feedback_count": client_feedback_count,
        "wire_feedback_count": wire_feedback_count,
        "failures": failures,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("reports", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--feedback-log", type=Path)
    args = parser.parse_args()

    reports = [json.loads(path.read_text(encoding="utf-8")) for path in args.reports]
    wire_feedback_count = None
    if args.feedback_log:
        wire_feedback_count = args.feedback_log.read_text(
            encoding="utf-8", errors="replace"
        ).count("progress:")
    result = audit_reports(reports, wire_feedback_count)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
