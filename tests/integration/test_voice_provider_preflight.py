import importlib.util
import sys
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "voice_provider_preflight.py"
spec = importlib.util.spec_from_file_location("voice_provider_preflight", SCRIPT)
preflight = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = preflight
spec.loader.exec_module(preflight)


def write_config(path: Path, *, keyword: str = "", silero: str = "") -> Path:
    path.write_text(
        f"""
silero_vad:
  ros__parameters:
    use_onnx: true
    model_path: "{silero}"
keyword_wake:
  ros__parameters:
    sherpa_tokens: "{keyword}/tokens.txt"
    sherpa_encoder: "{keyword}/encoder.onnx"
    sherpa_decoder: "{keyword}/decoder.onnx"
    sherpa_joiner: "{keyword}/joiner.onnx"
    sherpa_keywords_file: "{keyword}/keywords.txt"
    openwakeword_models: [""]
    livekit_wakeword_models: [""]
""".lstrip(),
        encoding="utf-8",
    )
    return path


def finder(*available):
    names = set(available)
    return lambda module: object() if module in names else None


def test_energy_and_text_wake_do_not_need_optional_dependencies(tmp_path):
    report = preflight.check_voice_providers(
        mode="offline",
        vad_provider="energy",
        kws_provider="none",
        config_path=write_config(tmp_path / "agent.yaml"),
        module_finder=finder(),
    )

    assert report.ok
    assert report.blockers == ()


def test_silero_requires_python_and_onnxruntime_packages(tmp_path):
    report = preflight.check_voice_providers(
        mode="online",
        vad_provider="silero",
        kws_provider="none",
        config_path=write_config(tmp_path / "agent.yaml"),
        module_finder=finder(),
    )

    assert not report.ok
    assert "vad:silero_vad_package_missing" in report.blockers
    assert "vad:onnxruntime_package_missing" in report.blockers


def test_sherpa_kws_requires_package_and_all_model_files(tmp_path):
    config = write_config(tmp_path / "agent.yaml", keyword=str(tmp_path / "missing"))

    report = preflight.check_voice_providers(
        mode="offline",
        vad_provider="energy",
        kws_provider="sherpa",
        config_path=config,
        module_finder=finder(),
    )

    assert not report.ok
    assert "kws:sherpa_onnx_package_missing" in report.blockers
    assert any(item.startswith("kws:sherpa_encoder_missing:") for item in report.blockers)


def test_openwakeword_without_custom_models_is_allowed_but_warns(tmp_path):
    report = preflight.check_voice_providers(
        mode="online",
        vad_provider="energy",
        kws_provider="openwakeword",
        config_path=write_config(tmp_path / "agent.yaml"),
        module_finder=finder("openwakeword.model"),
    )

    assert report.ok
    assert "kws:openwakeword_using_default_models" in report.warnings


def test_livekit_requires_custom_model_paths(tmp_path):
    report = preflight.check_voice_providers(
        mode="online",
        vad_provider="energy",
        kws_provider="livekit",
        config_path=write_config(tmp_path / "agent.yaml"),
        module_finder=finder("livekit.wakeword"),
    )

    assert not report.ok
    assert "kws:livekit_wakeword_models_empty" in report.blockers


def test_format_report_explains_blockers(tmp_path):
    report = preflight.check_voice_providers(
        mode="online",
        vad_provider="silero",
        kws_provider="none",
        config_path=write_config(tmp_path / "agent.yaml"),
        module_finder=finder(),
    )

    rendered = preflight.format_report(report)

    assert "BLOCKED: voice provider preflight" in rendered
    assert "vad:silero_vad_package_missing" in rendered
