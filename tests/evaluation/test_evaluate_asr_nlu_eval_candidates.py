#!/usr/bin/env python3
"""ASR/NLU 候选评估集的临时回归评估测试。"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "tools" / "evaluation" / "evaluate_asr_nlu_eval_candidates.py"


def _write_jsonl(path: Path, rows: list[dict]) -> None:
    path.write_text(
        "\n".join(json.dumps(row, ensure_ascii=False) for row in rows) + "\n",
        encoding="utf-8",
    )


def test_candidate_eval_report_scores_parser_against_review_candidates(tmp_path):
    """候选集即使尚未合入正式 eval，也应能先跑 parser accuracy。"""

    candidates = tmp_path / "asr_nlu_eval_candidates.jsonl"
    report_path = tmp_path / "candidate_eval_report.json"
    _write_jsonl(
        candidates,
        [
            {
                "schema_version": 1,
                "id": "real_asr_pass",
                "text": "向右转然后向前走一秒",
                "expected_actions": [
                    {"name": "turn", "arguments": {"angular_z": -0.6, "duration_s": 2.6}},
                    {"name": "move", "arguments": {"linear_x": 0.2, "duration_s": 1.0}},
                ],
                "tags": ["real_asr", "needs_review", "multi_command"],
            },
            {
                "schema_version": 1,
                "id": "real_asr_fail",
                "suggested_eval_case": {
                    "id": "real_asr_fail",
                    "text": "向前走一秒",
                    "expected_actions": [
                        {
                            "name": "move",
                            "arguments": {"linear_x": 0.2, "duration_s": 2.0},
                        }
                    ],
                    "tags": ["real_asr", "needs_review", "move"],
                },
            },
        ],
    )

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--input",
            str(candidates),
            "--output",
            str(report_path),
            "--minimum",
            "0.5",
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=True,
    )

    summary = json.loads(result.stdout)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert summary["status"] == "PASS"
    assert summary["passed"] == 1
    assert summary["total"] == 2
    assert summary["accuracy"] == 0.5
    assert report["failed_cases"][0]["id"] == "real_asr_fail"
    assert report["failed_cases"][0]["failure_type"] == "action_argument_mismatch"
    assert report["failure_analysis"]["failure_counts"] == {
        "action_argument_mismatch": 1
    }
    plan = report["failure_analysis"]["improvement_plan"]
    assert any(item["area"] == "slots" for item in plan)
    assert any(item["area"] == "eval_dataset" for item in plan)
    assert report["cases"][0]["source"] == "nlu"
