import os
from typing import Dict, Iterator, List

from .base import LlmProvider


class OpenAiCompatibleLlm(LlmProvider):
    """Streaming chat adapter for DashScope's OpenAI-compatible endpoint."""

    def __init__(
        self,
        model: str = "qwen-plus",
        base_url: str = "https://dashscope.aliyuncs.com/compatible-mode/v1",
        temperature: float = 0.2,
    ):
        self.model = model
        self.base_url = os.getenv("DASHSCOPE_BASE_URL", base_url)
        self.temperature = temperature

    def stream(self, messages: List[Dict[str, str]]) -> Iterator[str]:
        api_key = os.getenv("DASHSCOPE_API_KEY")
        if not api_key:
            raise RuntimeError("DASHSCOPE_API_KEY is required in online mode")
        try:
            from openai import OpenAI
        except ImportError as exc:
            raise RuntimeError("install requirements.txt for online LLM") from exc

        client = OpenAI(api_key=api_key, base_url=self.base_url)
        response = client.chat.completions.create(
            model=self.model,
            messages=messages,
            temperature=self.temperature,
            stream=True,
        )
        for item in response:
            if not item.choices:
                continue
            content = item.choices[0].delta.content
            if content:
                yield content

