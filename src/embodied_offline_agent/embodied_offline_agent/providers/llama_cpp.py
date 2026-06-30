from openai import OpenAI


class LlamaCppLlm:
    """Thin adapter around llama-server's OpenAI-compatible streaming API."""

    def __init__(self, base_url: str, model: str, temperature: float, max_tokens: int):
        self._client = OpenAI(base_url=base_url, api_key="local-offline")
        self._model = model
        self._temperature = temperature
        self._max_tokens = max_tokens

    def stream(self, messages):
        response = self._client.chat.completions.create(
            model=self._model,
            messages=messages,
            temperature=self._temperature,
            max_tokens=self._max_tokens,
            stream=True,
            extra_body={"chat_template_kwargs": {"enable_thinking": False}},
        )
        for event in response:
            delta = event.choices[0].delta.content if event.choices else None
            if delta:
                yield delta
