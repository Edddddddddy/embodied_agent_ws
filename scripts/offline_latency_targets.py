#!/usr/bin/env python3
"""Offline latency gate for llama.cpp first token and local TTS first audio."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
import urllib.request
from pathlib import Path
from statistics import median


WORKSPACE = Path(__file__).resolve().parents[1]
LLM_FIRST_TOKEN_TARGET_MS = 1000.0
# 离线 Sherpa provider 当前返回整句 PCM；600ms 是短反馈整句合成门槛。
# 在线流式 TTS 的 300ms 首包目标属于另一条链路，不能混用。
TTS_SYNTHESIS_TARGET_MS = 600.0


def _p95(values: list[float]) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[max(0, math.ceil(len(ordered) * 0.95) - 1)]


def summarize_llm_samples(
    samples: list[dict[str, object]],
    *,
    warmup: dict[str, object],
    target_ms: float,
) -> dict[str, object]:
    """将启动预热与用户可见 warm turn 分开，避免混淆冷启动和交互延迟。"""
    first_tokens = [
        float(item["first_token_ms"])
        for item in samples
        if isinstance(item.get("first_token_ms"), (int, float))
    ]
    decode_rates = [
        float(item["decode_tokens_per_s"])
        for item in samples
        if isinstance(item.get("decode_tokens_per_s"), (int, float))
    ]
    p95 = _p95(first_tokens)
    return {
        "measurement_kind": "warm_agent_turn_after_prefix_warmup",
        # 兼容 showcase claim 的既有字段；这里明确采用 warm turn P95。
        "first_token_ms": None if p95 is None else round(p95, 2),
        "median_first_token_ms": (
            None if not first_tokens else round(median(first_tokens), 2)
        ),
        "p95_first_token_ms": None if p95 is None else round(p95, 2),
        "median_decode_tokens_per_s": (
            None if not decode_rates else round(median(decode_rates), 2)
        ),
        "target_ms": target_ms,
        "warmup": warmup,
        "samples": samples,
        "ok": p95 is not None and p95 <= target_ms,
    }


def _ensure_repo_import_path() -> None:
    for relative in ("src/embodied_offline_agent", "src/embodied_online_agent"):
        path = str(WORKSPACE / relative)
        if path not in sys.path:
            sys.path.insert(0, path)


def post_llama_stream(base_url: str, timeout_s: float) -> dict[str, object]:
    request = urllib.request.Request(
        f"{base_url.rstrip('/')}/v1/chat/completions",
        data=json.dumps(
            {
                "model": "Qwen3-0.6B-Q8_0.gguf",
                "messages": [
                    {"role": "system", "content": "你是机器人动作解析助手。/no_think"},
                    {"role": "user", "content": "只回答：好的 /no_think"},
                ],
                "temperature": 0.0,
                "max_tokens": 8,
                "stream": True,
                "chat_template_kwargs": {"enable_thinking": False},
                "seed": 42,
            }
        ).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    started = time.perf_counter()
    first_token_ms = None
    parts: list[str] = []
    with urllib.request.urlopen(request, timeout=timeout_s) as response:
        for raw_line in response:
            line = raw_line.decode("utf-8", errors="replace").strip()
            if not line.startswith("data:"):
                continue
            data = line.removeprefix("data:").strip()
            if data == "[DONE]":
                break
            event = json.loads(data)
            for choice in event.get("choices") or []:
                token = (choice.get("delta") or {}).get("content")
                if not token:
                    continue
                if first_token_ms is None:
                    first_token_ms = (time.perf_counter() - started) * 1000.0
                parts.append(token)
    if first_token_ms is None:
        raise RuntimeError("llama.cpp stream returned no text token")
    return {
        "text": "".join(parts),
        "first_token_ms": round(first_token_ms, 2),
        "target_ms": LLM_FIRST_TOKEN_TARGET_MS,
        "ok": first_token_ms <= LLM_FIRST_TOKEN_TARGET_MS,
    }


def measure_llm_runtime(
    base_url: str,
    timeout_s: float,
    *,
    system_prompt: Path,
    sample_count: int = 3,
    history_turns: int = 3,
) -> dict[str, object]:
    """使用真实 Agent prompt、有限历史和 KV warmup 测量稳态首 token。"""
    _ensure_repo_import_path()
    from embodied_offline_agent.providers.llama_cpp import LlamaCppLlm

    system = system_prompt.read_text(encoding="utf-8") + "\n/no_think"
    seed_turns = [
        ("你好", "<speech>你好，我是小智。</speech>"),
        ("你是谁", "<speech>我是机器人语音助手小智。</speech>"),
        ("请简短回答", "<speech>好的。</speech>"),
    ]
    bounded_turns = max(0, min(int(history_turns), len(seed_turns)))
    history: list[dict[str, str]] = []
    for user, assistant in seed_turns[-bounded_turns:]:
        history.extend(
            [
                {"role": "user", "content": user},
                {"role": "assistant", "content": assistant},
            ]
        )

    api_base = base_url.rstrip("/")
    if not api_base.endswith("/v1"):
        api_base += "/v1"
    llm = LlamaCppLlm(
        api_base,
        "Qwen3-0.6B-Q8_0.gguf",
        0.0,
        32,
        seed=42,
        timeout_s=timeout_s,
        max_retries=0,
    )

    def messages(user_text: str) -> list[dict[str, str]]:
        return [
            {"role": "system", "content": system},
            *[dict(item) for item in history],
            {"role": "user", "content": user_text},
        ]

    llm.warmup(messages("只回复：就绪。"))
    warmup = llm.last_metrics
    samples: list[dict[str, object]] = []
    inputs = ("你是谁", "请简短回答。", "现在可以继续。")
    for index in range(max(1, int(sample_count))):
        user_text = inputs[index % len(inputs)]
        output = "".join(llm.stream(messages(user_text)))
        metrics = llm.last_metrics
        samples.append(
            {
                "index": index + 1,
                "history_turns": len(history) // 2,
                "first_token_ms": metrics.get("first_token_ms"),
                "total_ms": metrics.get("total_ms"),
                "prompt_tokens": metrics.get("prompt_tokens"),
                "completion_tokens": metrics.get("completion_tokens"),
                "decode_tokens_per_s": metrics.get("decode_tokens_per_s"),
            }
        )
        history.extend(
            [
                {"role": "user", "content": user_text},
                # 保存原始模型输出，模拟 Agent 的 KV 前缀复用策略。
                {"role": "assistant", "content": output},
            ]
        )
        if bounded_turns:
            history = history[-bounded_turns * 2 :]
        else:
            history = []
    return summarize_llm_samples(
        samples,
        warmup=warmup,
        target_ms=LLM_FIRST_TOKEN_TARGET_MS,
    )


def create_tts(provider: str):
    _ensure_repo_import_path()
    if provider == "sherpa":
        from embodied_offline_agent.providers.sherpa_tts import SherpaVitsTts

        return SherpaVitsTts(
            str(WORKSPACE / "models" / "vits-melo-tts-zh_en"),
            num_threads=2,
            speaker_id=0,
            speed=1.0,
        )
    if provider == "summer":
        from embodied_offline_agent.providers.summer_tts import SummerTts

        return SummerTts(
            str(WORKSPACE / "third_party" / "SummerTTS" / "build" / "tts_test"),
            str(
                WORKSPACE
                / "third_party"
                / "SummerTTS"
                / "models"
                / "single_speaker_fast.bin"
            ),
            timeout_s=30.0,
        )
    raise ValueError(f"unsupported tts provider: {provider}")


def measure_tts(provider: str, text: str, warmup: bool) -> dict[str, object]:
    started = time.perf_counter()
    tts = create_tts(provider)
    load_ms = (time.perf_counter() - started) * 1000.0
    if warmup:
        # 模型加载后的首句才是常驻 provider 的在线服务延迟；预热句不计入目标。
        tts.synthesize("好。")
    synth_started = time.perf_counter()
    pcm = tts.synthesize(text)
    synthesis_ms = (time.perf_counter() - synth_started) * 1000.0
    return {
        "provider": provider,
        "text": text,
        "load_ms": round(load_ms, 2),
        # SherpaVitsTts.synthesize() 返回整句 PCM；这里不能把函数返回时间冒充流式首包。
        "measurement_kind": "full_utterance_synthesis",
        "first_audio_supported": False,
        "synthesis_ms": round(synthesis_ms, 2),
        "target_ms": TTS_SYNTHESIS_TARGET_MS,
        "pcm_bytes": len(pcm),
        "sample_rate": getattr(tts, "sample_rate", None),
        "warmup": warmup,
        "ok": synthesis_ms <= TTS_SYNTHESIS_TARGET_MS and len(pcm) > 0,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://127.0.0.1:8080")
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument(
        "--system-prompt",
        type=Path,
        default=WORKSPACE
        / "src"
        / "embodied_agent_core"
        / "prompts"
        / "system_prompt_zh.txt",
    )
    parser.add_argument("--llm-samples", type=int, default=3)
    parser.add_argument("--llm-history-turns", type=int, default=3)
    parser.add_argument("--tts-provider", default="sherpa", choices=["sherpa", "summer"])
    parser.add_argument("--tts-text", default="好的。")
    parser.add_argument("--no-llm", action="store_true")
    parser.add_argument("--no-tts", action="store_true")
    parser.add_argument("--no-tts-warmup", action="store_true")
    parser.add_argument("--check", action="store_true")
    parser.add_argument(
        "--output",
        default="logs/offline_latency_report.json",
        help="保存可复查的实际延迟报告；传空字符串可禁用写文件",
    )
    args = parser.parse_args()

    report: dict[str, object] = {
        "schema_version": 1,
        "scenario": "offline_runtime_latency",
        "targets": {
            "llm_first_token_ms": LLM_FIRST_TOKEN_TARGET_MS,
            "tts_full_synthesis_ms": TTS_SYNTHESIS_TARGET_MS,
        }
    }
    ok = True
    if not args.no_llm:
        report["llm"] = measure_llm_runtime(
            args.base_url,
            args.timeout_s,
            system_prompt=args.system_prompt,
            sample_count=args.llm_samples,
            history_turns=args.llm_history_turns,
        )
        ok = ok and bool(report["llm"]["ok"])  # type: ignore[index]
    if not args.no_tts:
        report["tts"] = measure_tts(
            args.tts_provider, args.tts_text, warmup=not args.no_tts_warmup
        )
        ok = ok and bool(report["tts"]["ok"])  # type: ignore[index]
    report["ok"] = ok
    if args.output:
        output = Path(args.output).expanduser()
        if not output.is_absolute():
            output = WORKSPACE / output
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if args.check and not ok:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
