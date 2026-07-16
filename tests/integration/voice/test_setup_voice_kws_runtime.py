#!/usr/bin/env python3
"""声学唤醒 KWS 可选依赖安装脚本的 dry-run 验收。"""

from __future__ import annotations

import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "setup_voice_kws_runtime.sh"


def test_setup_voice_kws_runtime_dry_run_openwakeword_profile():
    result = subprocess.run(
        ["bash", str(SCRIPT), "openwakeword", "--dry-run"],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    assert "profile=openwakeword" in result.stdout
    assert "DRY RUN" in result.stdout
    assert "embodied_voice_frontend[kws]" in result.stdout
    assert "KWS_PROVIDER=openwakeword" in result.stdout
    assert "voice_provider_preflight.py" in result.stdout


def test_setup_voice_kws_runtime_dry_run_sherpa_profile_reuses_asr_assets():
    script_text = SCRIPT.read_text(encoding="utf-8")
    result = subprocess.run(
        ["bash", str(SCRIPT), "sherpa", "--dry-run"],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    assert "profile=sherpa" in result.stdout
    assert "setup_sherpa_asr_runtime.sh" in result.stdout
    assert "SHERPA_KWS_KEYWORDS_FILE" in result.stdout
    assert "SHERPA_KWS_ENV" in result.stdout
    assert "KWS_PROVIDER=sherpa" in result.stdout
    assert "--sherpa-tokens" in result.stdout
    assert "--sherpa-encoder" in result.stdout
    assert "--sherpa-keywords-file" in result.stdout
    assert "source logs/sherpa_kws.env" in result.stdout
    assert "小 智" in script_text
    assert "你 好 小 智" in script_text


def test_setup_voice_kws_runtime_dry_run_all_profile_includes_openwakeword_and_sherpa():
    result = subprocess.run(
        ["bash", str(SCRIPT), "all", "--dry-run"],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    assert "profile=all" in result.stdout
    assert "embodied_voice_frontend[kws,livekit-kws]" in result.stdout
    assert "setup_sherpa_asr_runtime.sh" in result.stdout


def test_setup_voice_kws_runtime_rejects_unknown_profile():
    result = subprocess.run(
        ["bash", str(SCRIPT), "unknown", "--dry-run"],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    assert result.returncode == 2
    assert "Usage:" in result.stderr
