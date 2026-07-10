from types import SimpleNamespace

import pytest

from embodied_offline_agent.providers.llama_cpp import LlamaCppError, LlamaCppLlm


def event(text):
    return SimpleNamespace(
        choices=[SimpleNamespace(delta=SimpleNamespace(content=text))]
    )


class FakeCompletions:
    def __init__(self, parent):
        self._parent = parent

    def create(self, **kwargs):
        self._parent.calls.append(kwargs)
        item = self._parent.responses.pop(0)
        if isinstance(item, Exception):
            raise item
        return item


class FakeClient:
    def __init__(self, responses):
        self.responses = list(responses)
        self.calls = []
        self.chat = SimpleNamespace(completions=FakeCompletions(self))


def factory_for(client):
    def _factory(**_kwargs):
        client.factory_kwargs = _kwargs
        return client

    return _factory


def test_stream_yields_tokens_and_records_metrics():
    client = FakeClient([[event("好"), event("的")]])
    llm = LlamaCppLlm(
        "http://127.0.0.1:8080/v1",
        "Qwen3-0.6B-Q8_0.gguf",
        0.1,
        16,
        client_factory=factory_for(client),
    )

    assert "".join(llm.stream([{"role": "user", "content": "向前走"}])) == "好的"

    call = client.calls[0]
    assert call["stream"] is True
    assert call["model"] == "Qwen3-0.6B-Q8_0.gguf"
    assert call["extra_body"]["chat_template_kwargs"]["enable_thinking"] is False
    assert llm.last_metrics["token_count"] == 2
    assert llm.last_metrics["first_token_ms"] is not None
    assert llm.last_metrics["tokens_per_s"] is not None


def test_stream_retries_before_any_token_is_emitted():
    client = FakeClient([RuntimeError("server warming"), [event("ok")]])
    llm = LlamaCppLlm(
        "http://127.0.0.1:8080/v1",
        "model.gguf",
        0.0,
        8,
        max_retries=1,
        client_factory=factory_for(client),
    )

    assert list(llm.stream([{"role": "user", "content": "ping"}])) == ["ok"]
    assert len(client.calls) == 2
    assert llm.last_metrics["retry_count"] == 1


def test_stream_does_not_retry_after_partial_output():
    def broken_stream():
        yield event("半")
        raise RuntimeError("stream dropped")

    client = FakeClient([broken_stream(), [event("不应出现")]])
    llm = LlamaCppLlm(
        "http://127.0.0.1:8080/v1",
        "model.gguf",
        0.0,
        8,
        max_retries=1,
        client_factory=factory_for(client),
    )

    with pytest.raises(LlamaCppError) as exc:
        list(llm.stream([{"role": "user", "content": "ping"}]))

    assert "stream dropped" in str(exc.value)
    assert len(client.calls) == 1
    assert llm.last_metrics["token_count"] == 1


def test_stream_error_message_contains_base_url_and_model():
    client = FakeClient([RuntimeError("boom")])
    llm = LlamaCppLlm(
        "http://127.0.0.1:8080/v1",
        "model.gguf",
        0.0,
        8,
        max_retries=0,
        client_factory=factory_for(client),
    )

    with pytest.raises(LlamaCppError) as exc:
        list(llm.stream([{"role": "user", "content": "ping"}]))

    message = str(exc.value)
    assert "http://127.0.0.1:8080/v1" in message
    assert "model.gguf" in message


def test_warmup_drains_stream_and_returns_observable_metrics():
    client = FakeClient([[event("就绪")]])
    llm = LlamaCppLlm(
        "http://127.0.0.1:8080/v1",
        "model.gguf",
        0.0,
        8,
        client_factory=factory_for(client),
    )

    report = llm.warmup([{"role": "system", "content": "固定系统提示"}])

    assert report["text"] == "就绪"
    assert report["metrics"]["token_count"] == 1
    assert len(client.calls) == 1
