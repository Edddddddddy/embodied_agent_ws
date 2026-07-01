from pathlib import Path

from embodied_offline_agent.providers.sherpa_asr import SherpaZipformerAsr


class FakeRecognizer:
    def create_stream(self):
        return object()


def test_zipformer_uses_contextual_biasing(monkeypatch, tmp_path: Path):
    captured = {}

    def fake_from_transducer(**kwargs):
        captured.update(kwargs)
        return FakeRecognizer()

    monkeypatch.setattr(
        "sherpa_onnx.OnlineRecognizer.from_transducer", fake_from_transducer
    )
    hotwords = tmp_path / "hotwords.txt"
    hotwords.write_text("小智\n向前走\n", encoding="utf-8")

    SherpaZipformerAsr(
        str(tmp_path), 16000, 2,
        hotwords_file=str(hotwords), hotwords_score=2.5,
    )

    assert captured["decoding_method"] == "modified_beam_search"
    assert captured["hotwords_file"] == str(hotwords)
    assert captured["hotwords_score"] == 2.5
    assert captured["modeling_unit"] == "cjkchar"
