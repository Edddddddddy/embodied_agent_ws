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
    recommendations: tuple[str, ...] = ()
    mature_vad_active: bool = False
    vad_maturity: str = "unknown"
    stability_summary: str = ""

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
    try:
        return finder(module) is not None
    except ModuleNotFoundError:
        # importlib.util.find_spec("pkg.submodule") 会在父包不存在时抛异常，而不是返回 None。
        # preflight 面向现场排障，缺依赖应该变成可读 blocker，不能直接 traceback。
        return False


def _append_once(items: list[str], item: str) -> None:
    if item not in items:
        items.append(item)


def _vad_stability_fields(vad_provider: str) -> dict[str, object]:
    if vad_provider == "silero":
        return {
            "mature_vad_active": True,
            "vad_maturity": "mature_acoustic_silero",
            "stability_summary": "Silero VAD is active; endpointing no longer depends only on energy threshold.",
        }
    if vad_provider == "webrtc":
        return {
            "mature_vad_active": True,
            "vad_maturity": "mature_acoustic_webrtc",
            "stability_summary": "WebRTC VAD is active; endpointing uses a mature acoustic VAD fallback.",
        }
    if vad_provider == "energy":
        return {
            "mature_vad_active": False,
            "vad_maturity": "energy_fallback",
            "stability_summary": "Energy VAD fallback is launchable but still environment-sensitive.",
        }
    return {
        "mature_vad_active": False,
        "vad_maturity": "unknown",
        "stability_summary": f"Unknown VAD provider: {vad_provider}",
    }


def _build_report(
    *,
    vad_provider: str,
    kws_provider: str,
    config: Path,
    blockers: list[str],
    warnings: list[str],
    recommendations: list[str],
) -> ProviderPreflightReport:
    return ProviderPreflightReport(
        vad_provider=vad_provider,
        kws_provider=kws_provider,
        config_path=str(config),
        blockers=tuple(blockers),
        warnings=tuple(warnings),
        recommendations=tuple(recommendations),
        **_vad_stability_fields(vad_provider),
    )


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


def _webrtc_blockers(
    *,
    module_finder: Callable[[str], object | None],
) -> tuple[str, ...]:
    blockers: list[str] = []
    if not _has_module("webrtcvad", module_finder):
        blockers.append("vad:webrtcvad_package_missing")
    return tuple(blockers)


def check_voice_providers(
    *,
    mode: str,
    vad_provider: str,
    kws_provider: str,
    config_path: Path | None = None,
    vad_overrides: Mapping[str, object] | None = None,
    keyword_overrides: Mapping[str, object] | None = None,
    require_mature_vad: bool = False,
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
    recommendations: list[str] = []

    if vad == "auto":
        silero_blockers = _silero_blockers(silero, module_finder=module_finder)
        if not silero_blockers:
            vad = "silero"
            warnings.append("vad:auto_selected:silero")
        else:
            webrtc_blockers = _webrtc_blockers(module_finder=module_finder)
            if not webrtc_blockers:
                vad = "webrtc"
                warnings.append(
                    "vad:auto_fallback:webrtc:"
                    + ",".join(silero_blockers)
                )
                _append_once(
                    recommendations,
                    "bash scripts/setup_voice_vad_runtime.sh all",
                )
            else:
                vad = "energy"
                warnings.append(
                    "vad:auto_fallback:energy:"
                    + ",".join((*silero_blockers, *webrtc_blockers))
                )
                _append_once(
                    recommendations,
                    "bash scripts/setup_voice_vad_runtime.sh webrtc",
                )

    if vad not in {"energy", "silero", "webrtc"}:
        warnings.append(f"vad:unknown_provider:{vad}")
    if vad == "silero":
        silero_blockers = _silero_blockers(silero, module_finder=module_finder)
        blockers.extend(silero_blockers)
        if silero_blockers:
            _append_once(
                recommendations,
                "bash scripts/setup_voice_vad_runtime.sh silero",
            )
    if vad == "webrtc":
        webrtc_blockers = _webrtc_blockers(module_finder=module_finder)
        blockers.extend(webrtc_blockers)
        if webrtc_blockers:
            _append_once(
                recommendations,
                "bash scripts/setup_voice_vad_runtime.sh webrtc",
            )

    if require_mature_vad and vad not in {"silero", "webrtc"}:
        blockers.append("vad:mature_provider_required")
        _append_once(recommendations, "bash scripts/setup_voice_vad_runtime.sh all")

    if kws in {"none", "disabled", "mock_text"}:
        return _build_report(
            vad_provider=vad,
            kws_provider=kws,
            config=config,
            blockers=blockers,
            warnings=warnings,
            recommendations=recommendations,
        )

    if kws == "sherpa":
        if not _has_module("sherpa_onnx", module_finder):
            blockers.append("kws:sherpa_onnx_package_missing")
            _append_once(recommendations, "bash scripts/setup_voice_kws_runtime.sh sherpa")
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
        if any(blocker.startswith("kws:") for blocker in blockers):
            _append_once(recommendations, "bash scripts/setup_voice_kws_runtime.sh sherpa")
    elif kws == "openwakeword":
        openwakeword_available = _has_module("openwakeword.model", module_finder)
        if not openwakeword_available:
            blockers.append("kws:openwakeword_package_missing")
            _append_once(
                recommendations,
                "bash scripts/setup_voice_kws_runtime.sh openwakeword",
            )
        models = _truthy_paths(keyword.get("openwakeword_models", []))
        missing = _missing_paths(models)
        for path in missing:
            blockers.append(f"kws:openwakeword_model_missing:{path}")
        if not models and openwakeword_available:
            warnings.append("kws:openwakeword_using_default_models")
    elif kws == "livekit":
        if not _has_module("livekit.wakeword", module_finder):
            blockers.append("kws:livekit_wakeword_package_missing")
            _append_once(recommendations, "bash scripts/setup_voice_kws_runtime.sh livekit")
        models = _truthy_paths(keyword.get("livekit_wakeword_models", []))
        if not models:
            blockers.append("kws:livekit_wakeword_models_empty")
        for path in _missing_paths(models):
            blockers.append(f"kws:livekit_wakeword_model_missing:{path}")
    else:
        warnings.append(f"kws:unknown_provider:{kws}")

    return _build_report(
        vad_provider=vad,
        kws_provider=kws,
        config=config,
        blockers=blockers,
        warnings=warnings,
        recommendations=recommendations,
    )


def format_report(report: ProviderPreflightReport) -> str:
    status = "PASS" if report.ok else "BLOCKED"
    lines = [
        f"{status}: voice provider preflight",
        f"  vad_provider: {report.vad_provider}",
        f"  vad_maturity: {report.vad_maturity}",
        f"  mature_vad_active: {str(report.mature_vad_active).lower()}",
        f"  kws_provider: {report.kws_provider}",
        f"  config: {report.config_path}",
        f"  stability: {report.stability_summary}",
    ]
    if report.blockers:
        lines.append("  blockers:")
        for blocker in report.blockers:
            lines.append(f"    - {blocker}")
    if report.warnings:
        lines.append("  warnings:")
        for warning in report.warnings:
            lines.append(f"    - {warning}")
    if report.recommendations:
        lines.append("  recommendations:")
        for recommendation in report.recommendations:
            lines.append(f"    - {recommendation}")
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
    parser.add_argument(
        "--require-mature-vad",
        action="store_true",
        help="Fail if auto/provider resolution falls back to energy VAD.",
    )
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
        require_mature_vad=args.require_mature_vad,
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
