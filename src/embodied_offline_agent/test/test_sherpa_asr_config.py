import importlib
import sys
import types
from pathlib import Path

import numpy as np


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
    assert captured["max_active_paths"] == 16


def test_zipformer_commit_flushes_encoder_tail_before_finishing(monkeypatch, tmp_path: Path):
    events: list[tuple] = []

    class FakeStream:
        def accept_waveform(self, sample_rate, samples):
            events.append(
                ("accept_waveform", sample_rate, np.asarray(samples).copy())
            )

        def input_finished(self):
            events.append(("input_finished",))

    class CommitRecognizer:
        def __init__(self):
            self.stream_count = 0
            self.ready = [True, False]

        def create_stream(self):
            self.stream_count += 1
            events.append(("create_stream", self.stream_count))
            return FakeStream()

        def is_ready(self, _stream):
            return self.ready.pop(0)

        def decode_stream(self, _stream):
            events.append(("decode_stream",))

        def get_result(self, _stream):
            events.append(("get_result",))
            return types.SimpleNamespace(text="完整长句")

    recognizer = CommitRecognizer()
    fake_sherpa = types.SimpleNamespace(
        OnlineRecognizer=types.SimpleNamespace(
            from_transducer=lambda **_kwargs: recognizer
        )
    )
    monkeypatch.setitem(sys.modules, "sherpa_onnx", fake_sherpa)
    sys.modules.pop("embodied_offline_agent.providers.sherpa_asr", None)
    module = importlib.import_module("embodied_offline_agent.providers.sherpa_asr")

    finals: list[str] = []
    asr = module.SherpaZipformerAsr(
        str(tmp_path), 16_000, 2, tail_padding_s=0.66
    )
    asr.start(lambda _text: None, finals.append)
    events.clear()

    asr.commit()

    assert [event[0] for event in events] == [
        "accept_waveform",
        "input_finished",
        "decode_stream",
        "get_result",
        "create_stream",
    ]
    _, sample_rate, padding = events[0]
    assert sample_rate == 16_000
    assert padding.dtype == np.float32
    assert padding.shape == (10_560,)
    assert np.count_nonzero(padding) == 0
    assert finals == ["完整长句"]


def test_offline_node_forwards_configured_asr_tail_padding(monkeypatch):
    captured: dict[str, object] = {}

    class FakeAsr:
        def __init__(self, *args, **kwargs):
            captured["asr_args"] = args
            captured["asr_kwargs"] = kwargs

    class FakeLlm:
        def __init__(self, *args, **kwargs):
            captured["llm_args"] = args
            captured["llm_kwargs"] = kwargs

    asr_module_name = "embodied_offline_agent.providers.sherpa_asr"
    llm_module_name = "embodied_offline_agent.providers.llama_cpp"
    fake_asr_module = types.ModuleType(asr_module_name)
    fake_asr_module.SherpaZipformerAsr = FakeAsr
    fake_llm_module = types.ModuleType(llm_module_name)
    fake_llm_module.LlamaCppLlm = FakeLlm
    monkeypatch.setitem(sys.modules, asr_module_name, fake_asr_module)
    monkeypatch.setitem(sys.modules, llm_module_name, fake_llm_module)

    from embodied_offline_agent.offline_agent_node import OfflineAgentNode

    values = {
        "asr_model_dir": "/models/zipformer",
        "audio_sample_rate": 16_000,
        "asr_num_threads": 2,
        "asr_decoding_method": "modified_beam_search",
        "asr_hotwords_score": 3.0,
        "asr_max_active_paths": 4,
        "asr_modeling_unit": "cjkchar",
        "asr_tail_padding_s": 0.72,
        "llm_base_url": "http://127.0.0.1:8080/v1",
        "llm_model": "qwen",
        "llm_temperature": 0.1,
        "llm_max_tokens": 32,
        "llm_seed": 42,
        "llm_timeout_s": 10.0,
        "llm_max_retries": 1,
        "llm_first_token_warn_ms": 1_000.0,
    }
    node = object.__new__(OfflineAgentNode)
    node._mode = "offline"
    node._param = values.__getitem__
    node._hotwords_file = lambda: "/config/hotwords.txt"
    node._create_tts_provider = lambda: object()

    node._create_providers()

    assert captured["asr_kwargs"]["tail_padding_s"] == 0.72
