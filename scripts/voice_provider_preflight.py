#!/usr/bin/env python3
"""Preflight optional VAD/KWS providers before launching long voice control.

The continuous voice demo supports several optional providers. Without a preflight,
missing Python extras or empty model paths fail later inside ROS nodes, which is a
poor human-facing experience. This script is intentionally ROS-free so it can run
before microphone/Gazebo startup and in unit tests.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable, Iterable, Mapping

import yaml


ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class ProviderPreflightReport:
    vad_provider: str
    kws_provider: str
    config_path: str
    blockers: tuple[str, ...]
    warnings: tuple[str, ...]

    @property
    def ok(self) -> bool:
        return not self.blockers


def _default_config(mode: str) -> Path:
    if mode == "offline":
        return ROOT / "src" / "embodied_offline_agent" / "config" / "offline_agent.yaml"
    return ROOT / "src" / "embodied_online_agent" / "config" / "online_agent.yaml"


def _load_section(config_path: Path, section: str) -> dict:
    if not config_path.exists():
        return {}
    data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    node = data.get(section, {})
    params = node.get("ros__parameters", {}) if isinstance(node, dict) else {}
    return params if isinstance(params, dict) else {}


def _non_empty(value: object) -> bool:
    if value is None:
        return False
    if isinstance(value, str):
        return bool(value.strip())
    if isinstance(value, Iterable):
        return bool(_truthy_paths(value))
    return bool(str(value).strip())


def _merged_params(params: dict, overrides: Mapping[str, object] | None) -> dict:
    """Merge CLI/env overrides on top of YAML params.

    Empty override values are ignored intentionally: users can keep using YAML as the
    source of truth, while the continuous voice launcher can still pass optional env
    values without accidentally clearing model paths.
    """

    merged = dict(params)
    for name, value in (overrides or {}).items():
        if _non_empty(value):
            merged[name] = value
    return merged


def _bool_value(value: object, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "on"}:
        return True
    if text in {"0", "false", "no", "off"}:
        return False
    return default


def _truthy_paths(values: object) -> list[str]:
    if values is None:
        return []
    if isinstance(values, str):
        candidates = values.split(",")
    elif isinstance(values, Iterable):
        candidates = [str(value) for value in values]
    else:
        candidates = [str(values)]
    return [item for item in (candidate.strip() for candidate in candidates) if item]


def _missing_paths(paths: Iterable[str]) -> list[str]:
    missing = []
    for raw in paths:
        path = Path(os.path.expanduser(raw))
        if not path.exists():
            missing.append(raw)
    return missing


def _has_module(module: str, finder: Callable[[str], object | None]) -> bool:
    return finder(module) is not None


def _silero_blockers(
    silero: Mapping[str, object],
    *,
    module_finder: Callable[[str], object | None],
) -> tuple[str, ...]:
    blockers: list[str] = []
    if not _has_module("silero_vad", module_finder):
        blockers.append("vad:silero_vad_package_missing")
    if _bool_value(silero.get("use_onnx", True), True) and not _has_module(
        "onnxruntime", module_finder
    ):
        blockers.append("vad:onnxruntime_package_missing")
    model_path = str(silero.get("model_path", "") or "").strip()
    if model_path and _missing_paths([model_path]):
        blockers.append(f"vad:silero_model_missing:{model_path}")
    return tuple(blockers)


def check_voice_providers(
    *,
    mode: str,
    vad_provider: str,
    kws_provider: str,
    config_path: Path | None = None,
    vad_overrides: Mapping[str, object] | None = None,
    keyword_overrides: Mapping[str, object] | None = None,
    module_finder: Callable[[str], object | None] = importlib.util.find_spec,
) -> ProviderPreflightReport:
    config = config_path or _default_config(mode)
    requested_vad = (vad_provider or "auto").strip().lower()
    vad = requested_vad
    kws = (kws_provider or "none").strip().lower()
    silero = _merged_params(_load_section(config, "silero_vad"), vad_overrides)
    keyword = _merged_params(
        _load_section(config, "keyword_wake"), keyword_overrides
    )
    blockers: list[str] = []
    warnings: list[str] = []

    if vad == "auto":
        silero_blockers = _silero_blockers(silero, module_finder=module_finder)
        if silero_blockers:
            vad = "energy"
            warnings.append("vad:auto_fallback:energy:" + ",".join(silero_blockers))
        else:
            vad = "silero"
            warnings.append("vad:auto_selected:silero")

    if vad not in {"energy", "silero"}:
        warnings.append(f"vad:unknown_provider:{vad}")
    if vad == "silero":
        blockers.extend(_silero_blockers(silero, module_finder=module_finder))

    if kws in {"none", "disabled", "mock_text"}:
        return ProviderPreflightReport(vad, kws, str(config), tuple(blockers), tuple(warnings))

    if kws == "sherpa":
        if not _has_module("sherpa_onnx", module_finder):
            blockers.append("kws:sherpa_onnx_package_missing")
        required = [
            "sherpa_tokens",
            "sherpa_encoder",
            "sherpa_decoder",
            "sherpa_joiner",
            "sherpa_keywords_file",
        ]
        for name in required:
            value = str(keyword.get(name, "") or "").strip()
            if not value:
                blockers.append(f"kws:{name}_empty")
            elif _missing_paths([value]):
                blockers.append(f"kws:{name}_missing:{value}")
    elif kws == "openwakeword":
        if not _has_module("openwakeword.model", module_finder):
            blockers.append("kws:openwakeword_package_missing")
        models = _truthy_paths(keyword.get("openwakeword_models", []))
        missing = _missing_paths(models)
        for path in missing:
            blockers.append(f"kws:openwakeword_model_missing:{path}")
        if not models:
            warnings.append("kws:openwakeword_using_default_models")
    elif kws == "livekit":
        if not _has_module("livekit.wakeword", module_finder):
            blockers.append("kws:livekit_wakeword_package_missing")
        models = _truthy_paths(keyword.get("livekit_wakeword_models", []))
        if not models:
            blockers.append("kws:livekit_wakeword_models_empty")
        for path in _missing_paths(models):
            blockers.append(f"kws:livekit_wakeword_model_missing:{path}")
    else:
        warnings.append(f"kws:unknown_provider:{kws}")

    return ProviderPreflightReport(vad, kws, str(config), tuple(blockers), tuple(warnings))


def format_report(report: ProviderPreflightReport) -> str:
    status = "PASS" if report.ok else "BLOCKED"
    lines = [
        f"{status}: voice provider preflight",
        f"  vad_provider: {report.vad_provider}",
        f"  kws_provider: {report.kws_provider}",
        f"  config: {report.config_path}",
    ]
    if report.blockers:
        lines.append("  blockers:")
        for blocker in report.blockers:
            lines.append(f"    - {blocker}")
    if report.warnings:
        lines.append("  warnings:")
        for warning in report.warnings:
            lines.append(f"    - {warning}")
    if report.ok:
        lines.append("  next: provider configuration looks launchable.")
    else:
        lines.append("  next: install missing extras or fix model paths before launch.")
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("online", "offline"), default="offline")
    parser.add_argument("--vad-provider", default="auto")
    parser.add_argument("--kws-provider", default="none")
    parser.add_argument("--config", type=Path, default=None)
    parser.add_argument("--silero-model-path", default="")
    parser.add_argument("--silero-use-onnx", default="")
    parser.add_argument("--sherpa-tokens", default="")
    parser.add_argument("--sherpa-encoder", default="")
    parser.add_argument("--sherpa-decoder", default="")
    parser.add_argument("--sherpa-joiner", default="")
    parser.add_argument("--sherpa-keywords-file", default="")
    parser.add_argument("--openwakeword-models", default="")
    parser.add_argument("--livekit-wakeword-models", default="")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    report = check_voice_providers(
        mode=args.mode,
        vad_provider=args.vad_provider,
        kws_provider=args.kws_provider,
        config_path=args.config,
        vad_overrides={
            "model_path": args.silero_model_path,
            "use_onnx": args.silero_use_onnx,
        },
        keyword_overrides={
            "sherpa_tokens": args.sherpa_tokens,
            "sherpa_encoder": args.sherpa_encoder,
            "sherpa_decoder": args.sherpa_decoder,
            "sherpa_joiner": args.sherpa_joiner,
            "sherpa_keywords_file": args.sherpa_keywords_file,
            "openwakeword_models": args.openwakeword_models,
            "livekit_wakeword_models": args.livekit_wakeword_models,
        },
    )
    if args.json:
        payload = asdict(report)
        payload["ok"] = report.ok
        print(json.dumps(payload, ensure_ascii=False, indent=2))
    else:
        print(format_report(report))
    if not report.ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
