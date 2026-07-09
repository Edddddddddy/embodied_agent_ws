#!/usr/bin/env python3
"""离线展示报告证据审计测试。"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "audit_offline_showcase_evidence.py"


def _write_report(path: Path, *, latency_status: str = "not_run") -> None:
    report = {
        "schema_version": 1,
        "scenario": "offline_deployment_showcase",
        "model_inventory": {
            "ok": True,
            "total_size_mb": 1234.5,
            "items": [
                {"key": "llm_qwen3_0_6b_q8", "exists": True, "size_mb": 620.0},
                {"key": "asr_zipformer_encoder", "exists": True, "size_mb": 42.0},
            ],
        },
        "runtime_versions": {
            "ok": True,
            "items": [{"name": "llama.cpp", "expected": "pinned", "actual": "pinned", "ok": True}],
        },
        "instruction_parser": {
            "dataset": "training/robot_instruction_eval.jsonl",
            "passed": 39,
            "total": 39,
            "accuracy": 1.0,
            "failed_cases": [],
        },
        "latency": {"status": latency_status, "reason": "not measured in default report"},
        "asr_tts_benchmark": {"status": "not_run", "reason": "not measured in default report"},
    }
    path.write_text(json.dumps(report, ensure_ascii=False), encoding="utf-8")


def test_offline_evidence_audit_warns_when_latency_is_not_measured(tmp_path):
    report = tmp_path / "offline_showcase_report.json"
    output = tmp_path / "offline_evidence_audit.json"
    _write_report(report)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--input",
            str(report),
            "--output",
            str(output),
        ],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    summary = json.loads(result.stdout)
    audit = json.loads(output.read_text(encoding="utf-8"))
    assert summary["status"] == "PASS"
    assert audit["ok"] is True
    assert "latency:not_measured" in audit["warnings"]
    assert audit["evidence"]["instruction_parser"]["status"] == "proven"
    assert audit["evidence"]["latency"]["status"] == "missing"
    assert any("不要说" in item and "首 token" in item for item in audit["claim_guidance"])


def test_offline_evidence_audit_can_require_real_latency(tmp_path):
    report = tmp_path / "offline_showcase_report.json"
    _write_report(report)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--input",
            str(report),
            "--require-latency",
        ],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    summary = json.loads(result.stdout)
    assert result.returncode == 1
    assert summary["status"] == "FAIL"
    assert "latency:required_but_not_measured" in summary["blockers"]
