#!/usr/bin/env python3
"""Nav2 语音演示资产审计测试。"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "audit_nav2_demo_assets.py"


def test_nav2_demo_asset_audit_reports_required_assets(tmp_path):
    output = tmp_path / "nav2_demo_assets.json"
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
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
    assert audit["assets"]["places"]["status"] == "present"
    assert {"home", "door", "desk"} <= set(audit["assets"]["places"]["place_names"])
    assert audit["assets"]["rviz_config"]["status"] == "present"
    assert audit["assets"]["launch"]["status"] == "present"
    assert "map" in audit["assets"]["launch"]["arguments"]
    assert "rviz_config_file" in audit["assets"]["launch"]["arguments"]
    assert "map:uses_nav2_builtin_tb3_sandbox" in audit["warnings"]


def test_nav2_demo_asset_audit_can_require_local_map_and_world():
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--require-local-assets",
        ],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    summary = json.loads(result.stdout)
    assert result.returncode == 1
    assert summary["status"] == "FAIL"
    assert "map:local_asset_required" in summary["blockers"]
    assert "world:local_asset_required" in summary["blockers"]
