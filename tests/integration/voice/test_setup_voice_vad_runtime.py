#!/usr/bin/env python3
"""成熟 VAD 可选依赖安装脚本的 dry-run 验收。"""

from __future__ import annotations

import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "setup_voice_vad_runtime.sh"


def test_setup_voice_vad_runtime_dry_run_webrtc_profile():
    result = subprocess.run(
        ["bash", str(SCRIPT), "webrtc", "--dry-run"],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    assert "profile=webrtc" in result.stdout
    assert "DRY RUN" in result.stdout
    assert "embodied_voice_frontend[webrtc-vad]" in result.stdout
    assert "voice_provider_preflight.py" in result.stdout
    assert "--vad-provider auto" in result.stdout


def test_setup_voice_vad_runtime_dry_run_all_profile_includes_silero():
    result = subprocess.run(
        ["bash", str(SCRIPT), "all", "--dry-run"],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    assert "profile=all" in result.stdout
    assert "embodied_voice_frontend[webrtc-vad,silero-vad]" in result.stdout
    assert "onnxruntime" not in result.stderr


def test_setup_voice_vad_runtime_rejects_unknown_profile():
    result = subprocess.run(
        ["bash", str(SCRIPT), "unknown", "--dry-run"],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    assert result.returncode == 2
    assert "Usage:" in result.stderr
