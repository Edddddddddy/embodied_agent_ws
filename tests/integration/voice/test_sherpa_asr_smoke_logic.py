import importlib.util
import json
import sys
import types
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location(
    "sherpa_asr_smoke", ROOT / "scripts" / "sherpa_asr_smoke.py"
)
assert SPEC is not None and SPEC.loader is not None
SMOKE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SMOKE)


class FakeAsr:
    def __init__(self, *_args, **_kwargs):
        self._on_final = None

    def start(self, _on_partial, on_final):
        self._on_final = on_final

    def push_audio(self, _pcm16):
        return None

    def commit(self):
        self._on_final("昨天阿星")


def test_smoke_fails_when_transcript_misses_required_substring(monkeypatch, tmp_path):
    fake_provider = types.SimpleNamespace(SherpaZipformerAsr=FakeAsr)
    monkeypatch.setitem(
        sys.modules, "embodied_offline_agent.providers.sherpa_asr", fake_provider
    )
    monkeypatch.setattr(SMOKE, "assert_preflight", lambda *_args: None)
    monkeypatch.setattr(
        SMOKE, "read_pcm16_wav", lambda *_args: (bytes(3_200), 0.1)
    )
    args = types.SimpleNamespace(
        model_dir=str(tmp_path),
        wav=str(tmp_path / "test.wav"),
        sample_rate=16_000,
        num_threads=2,
        decoding_method="modified_beam_search",
        hotwords_file="",
        hotwords_score=3.0,
        max_active_paths=16,
        modeling_unit="cjkchar",
        chunk_samples=1_600,
        allow_empty=False,
        expected_substring="星期三",
    )

    with pytest.raises(SystemExit) as raised:
        SMOKE.run_smoke(args)

    report = json.loads(str(raised.value))
    assert report["ok"] is False
    assert report["text"] == "昨天阿星"
    assert report["expected_substring"] == "星期三"
    assert report["expected_substring_found"] is False


def test_acceptance_smoke_requires_the_long_wav_tail_phrase():
    acceptance = (ROOT / "scripts" / "acceptance_test.sh").read_text(
        encoding="utf-8"
    )

    assert 'scripts/sherpa_asr_smoke.py --expected-substring "星期三"' in acceptance


def test_smoke_cli_uses_the_verified_beam_width_by_default():
    args = SMOKE.build_parser().parse_args([])

    assert args.max_active_paths == 16
    assert args.expected_substring == ""
