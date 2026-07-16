import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "tools" / "evaluation" / "audit_summer_tts_cache_evidence.py"


def _write_probe(path: Path, *, cache_hit: bool = True, last_ms: float = 42.0) -> None:
    path.write_text(
        json.dumps(
            {
                "ok": True,
                "sample_rate": 24000,
                "pcm_bytes": 4096,
                "service_synthesize_ms": 1.5 if cache_hit else 800.0,
                "roundtrip_ms": last_ms,
                "cache_hit": cache_hit,
                "cache_hits": 1 if cache_hit else 0,
                "first_roundtrip_ms": 900.0,
                "last_roundtrip_ms": last_ms,
                "responses": [
                    {
                        "index": 1,
                        "ok": True,
                        "pcm_bytes": 4096,
                        "cache_hit": False,
                        "roundtrip_ms": 900.0,
                    },
                    {
                        "index": 2,
                        "ok": True,
                        "pcm_bytes": 4096,
                        "cache_hit": cache_hit,
                        "roundtrip_ms": last_ms,
                    },
                ],
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )


def test_summer_tts_cache_audit_passes_for_cached_short_feedback(tmp_path):
    probe = tmp_path / "summer_tts_service_probe.json"
    output = tmp_path / "summer_tts_cache_audit.json"
    _write_probe(probe, cache_hit=True, last_ms=38.0)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--input",
            str(probe),
            "--output",
            str(output),
            "--target-roundtrip-ms",
            "300",
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
    assert audit["evidence"]["cache_hit"] is True
    assert audit["evidence"]["last_roundtrip_ms"] == 38.0
    assert any("固定短反馈语" in item for item in audit["claim_guidance"])
    assert any("不要说" in item and "整句生成" in item for item in audit["claim_guidance"])


def test_summer_tts_cache_audit_fails_when_cache_misses(tmp_path):
    probe = tmp_path / "summer_tts_service_probe.json"
    output = tmp_path / "summer_tts_cache_audit.json"
    _write_probe(probe, cache_hit=False, last_ms=820.0)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--input",
            str(probe),
            "--output",
            str(output),
        ],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    summary = json.loads(result.stdout)
    audit = json.loads(output.read_text(encoding="utf-8"))
    assert result.returncode == 1
    assert summary["status"] == "FAIL"
    assert "cache:required_but_not_hit" in audit["blockers"]
    assert "latency:cache_roundtrip_above_300ms" in audit["blockers"]
