import time
from dataclasses import asdict, dataclass
from typing import Callable, Iterable

from openai import APIConnectionError, APIStatusError, APITimeoutError, OpenAI, OpenAIError


@dataclass
class LlamaCppMetrics:
    request_count: int = 0
    retry_count: int = 0
    token_count: int = 0
    first_token_ms: float | None = None
    total_ms: float | None = None
    tokens_per_s: float | None = None
    first_token_target_met: bool | None = None
    model: str = ""
    base_url: str = ""
    error: str | None = None

    def as_dict(self) -> dict:
        return asdict(self)


class LlamaCppError(RuntimeError):
    """Readable llama.cpp provider error surfaced to the ROS node logs."""


class LlamaCppLlm:
    """Adapter around llama-server's OpenAI-compatible streaming API."""

    def __init__(
        self,
        base_url: str,
        model: str,
        temperature: float,
        max_tokens: int,
        seed: int = 42,
        timeout_s: float = 30.0,
        max_retries: int = 1,
        first_token_warn_ms: float = 1000.0,
        client_factory: Callable[..., object] = OpenAI,
    ):
        self._base_url = base_url.rstrip("/")
        self._model = model
        self._temperature = temperature
        self._max_tokens = max_tokens
        self._seed = seed
        self._timeout_s = float(timeout_s)
        self._max_retries = max(0, int(max_retries))
        self._first_token_warn_ms = float(first_token_warn_ms)
        # openai-python 自带 HTTP/SSE 解析；这里把 client factory 注入出来，便于单测构造假流。
        self._client = client_factory(
            base_url=self._base_url,
            api_key="local-offline",
            timeout=self._timeout_s,
            max_retries=0,
        )
        self._last_metrics = LlamaCppMetrics(model=self._model, base_url=self._base_url)

    @property
    def last_metrics(self) -> dict:
        return self._last_metrics.as_dict()

    def warmup(self, messages: Iterable[dict]) -> dict:
        """Prime llama-server's model kernels and reusable system-prompt KV prefix."""
        text = "".join(self.stream(messages))
        return {"text": text, "metrics": self.last_metrics}

    def stream(self, messages: Iterable[dict]):
        message_list = list(messages)
        attempts = self._max_retries + 1
        for attempt in range(1, attempts + 1):
            metrics = LlamaCppMetrics(
                request_count=attempt,
                retry_count=attempt - 1,
                model=self._model,
                base_url=self._base_url,
            )
            started = time.perf_counter()
            emitted_any_token = False
            try:
                response = self._create_stream(message_list)
                for event in response:
                    delta = self._event_delta(event)
                    if not delta:
                        continue
                    now = time.perf_counter()
                    if metrics.first_token_ms is None:
                        metrics.first_token_ms = (now - started) * 1000.0
                    metrics.token_count += 1
                    emitted_any_token = True
                    yield delta
                self._finish_metrics(metrics, started)
                self._last_metrics = metrics
                return
            except Exception as exc:
                metrics.error = self._describe_error(exc)
                self._finish_metrics(metrics, started)
                self._last_metrics = metrics
                # 一旦已经向上游吐过 token，就不能静默重试，否则会把半截回复拼到新回复前面。
                if emitted_any_token or attempt >= attempts:
                    raise LlamaCppError(metrics.error) from exc

    def _create_stream(self, messages):
        return self._client.chat.completions.create(
            model=self._model,
            messages=list(messages),
            temperature=self._temperature,
            max_tokens=self._max_tokens,
            stream=True,
            extra_body={
                "chat_template_kwargs": {"enable_thinking": False},
                "seed": self._seed,
            },
        )

    @staticmethod
    def _event_delta(event) -> str | None:
        choices = getattr(event, "choices", None) or []
        if not choices:
            return None
        delta = getattr(choices[0], "delta", None)
        return getattr(delta, "content", None) if delta is not None else None

    def _finish_metrics(self, metrics: LlamaCppMetrics, started: float) -> None:
        metrics.total_ms = (time.perf_counter() - started) * 1000.0
        if metrics.token_count and metrics.total_ms and metrics.total_ms > 0:
            metrics.tokens_per_s = metrics.token_count / (metrics.total_ms / 1000.0)
        if metrics.first_token_ms is not None:
            metrics.first_token_target_met = metrics.first_token_ms <= self._first_token_warn_ms

    def _describe_error(self, exc: Exception) -> str:
        prefix = f"llama.cpp request failed: base_url={self._base_url}, model={self._model}"
        if isinstance(exc, APITimeoutError):
            return f"{prefix}, timeout_s={self._timeout_s}: {exc}"
        if isinstance(exc, APIConnectionError):
            return f"{prefix}: cannot connect to llama-server ({exc})"
        if isinstance(exc, APIStatusError):
            return f"{prefix}: HTTP {exc.status_code} ({exc.response.text})"
        if isinstance(exc, OpenAIError):
            return f"{prefix}: {exc}"
        return f"{prefix}: {type(exc).__name__}: {exc}"
