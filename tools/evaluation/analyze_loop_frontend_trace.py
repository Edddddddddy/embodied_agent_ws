#!/usr/bin/env python3
"""Aggregate Karto loop-candidate and scan-matcher trace into reviewable evidence."""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path
from typing import Any


def _matcher_passed(event: dict[str, Any]) -> bool:
    response = event.get("response")
    threshold = event.get("response_threshold")
    if response is None or threshold is None or float(response) <= float(threshold):
        return False
    if event.get("event") != "matcher_coarse_check":
        return True
    variance_threshold = event.get("variance_threshold")
    variance_x = event.get("variance_x")
    variance_y = event.get("variance_y")
    return (
        variance_threshold is not None
        and variance_x is not None
        and variance_y is not None
        and float(variance_x) < float(variance_threshold)
        and float(variance_y) < float(variance_threshold)
    )


def analyze_trace(path: Path) -> dict[str, object]:
    """Explain the deepest reached loop-closure stage without treating zero loops as a crash."""

    events: list[dict[str, Any]] = []
    parse_errors = 0
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            parse_errors += 1
            continue
        if not isinstance(event, dict) or event.get("schema_version") != 1:
            parse_errors += 1
            continue
        event["_line_number"] = line_number
        events.append(event)

    topologies = [event for event in events if event.get("event") == "candidate_topology"]
    coarse = [event for event in events if event.get("event") == "matcher_coarse_check"]
    fine = [event for event in events if event.get("event") == "matcher_fine_check"]
    coarse_passed = sum(_matcher_passed(event) for event in coarse)
    fine_passed = sum(_matcher_passed(event) for event in fine)
    begins = sum(event.get("event") == "begin_closure" for event in events)
    ends = sum(event.get("event") == "end_closure" for event in events)
    rejections = sum(event.get("event") == "matcher_rejected" for event in events)

    reasons = Counter(
        str(event.get("topology", {}).get("primary_reason", "missing"))
        for event in topologies
    )
    candidate_scans = sum(
        int(event.get("replicated_karto_rule", {}).get("candidate_chains", 0)) > 0
        for event in topologies
    )
    if not coarse:
        boundary = "candidate_generation"
    elif coarse_passed == 0:
        boundary = "coarse_response_or_variance"
    elif not fine or fine_passed == 0:
        boundary = "fine_response"
    elif ends == 0:
        boundary = "constraint_insertion"
    else:
        boundary = "accepted_loop"

    maxima = {
        field: max(
            (int(event.get("topology", {}).get(field, 0)) for event in topologies),
            default=0,
        )
        for field in (
            "historical_scan_count",
            "geometric_near_count",
            "near_linked_count",
            "eligible_unlinked_count",
            "maximum_eligible_chain_size",
            "trailing_chain_size",
        )
    }
    checks = {
        "trace_nonempty": bool(events),
        "topology_events_present": bool(topologies),
        "all_lines_parseable": parse_errors == 0,
        "closure_callbacks_balanced": begins == ends,
    }
    return {
        "schema_version": 1,
        "passed": all(checks.values()),
        "checks": checks,
        "failure_boundary": boundary,
        "events": {
            "valid": len(events),
            "parse_errors": parse_errors,
            "processed_topology_scans": len(topologies),
            "candidate_scans": candidate_scans,
            "coarse_checks": len(coarse),
            "coarse_passed": coarse_passed,
            "coarse_failed": len(coarse) - coarse_passed,
            "fine_checks": len(fine),
            "fine_passed": fine_passed,
            "fine_failed": len(fine) - fine_passed,
            "matcher_rejections": rejections,
            "closure_begins": begins,
            "closure_ends": ends,
        },
        "candidate_primary_reasons": dict(sorted(reasons.items())),
        "topology_maxima": maxima,
        "methodology": {
            "candidate_rule": "replication of Karto FindPossibleLoopClosure chain topology",
            "matcher_events": "Karto MapperLoopClosureListener callbacks",
            "claim_boundary": (
                "diagnostic evidence identifies the reached stage; it does not create or accept constraints"
            ),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = analyze_trace(args.trace)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"{'PASS' if report['passed'] else 'FAIL'}: loop frontend trace")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
