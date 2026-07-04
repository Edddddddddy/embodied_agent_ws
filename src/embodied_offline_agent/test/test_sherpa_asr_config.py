import importlib
import sys
import types
from pathlib import Path


class FakeRecognizer:
    def create_stream(self):
        return object()


def test_zipformer_uses_contextual_biasing(monkeypatch, tmp_path: Path):
    captured = {}

    def fake_from_transducer(**kwargs):
        captured.update(kwargs)
        return FakeRecognizer()

    # sherpa_onnx 是离线真实模型依赖，普通 CI/开发机可能没有安装。
    # 这里在系统边界注入 fake 模块，只验证本项目是否把 hotwords 参数正确传给 sherpa。
    fake_sherpa = types.SimpleNamespace(
        OnlineRecognizer=types.SimpleNamespace(from_transducer=fake_from_transducer)
    )
    monkeypatch.setitem(sys.modules, "sherpa_onnx", fake_sherpa)
    sys.modules.pop("embodied_offline_agent.providers.sherpa_asr", None)
    module = importlib.import_module("embodied_offline_agent.providers.sherpa_asr")

    hotwords = tmp_path / "hotwords.txt"
    hotwords.write_text("小智\n向前走\n", encoding="utf-8")

    module.SherpaZipformerAsr(
        str(tmp_path), 16000, 2,
        hotwords_file=str(hotwords), hotwords_score=2.5,
    )

    assert captured["decoding_method"] == "modified_beam_search"
    assert captured["hotwords_file"] == str(hotwords)
    assert captured["hotwords_score"] == 2.5
    assert captured["modeling_unit"] == "cjkchar"
