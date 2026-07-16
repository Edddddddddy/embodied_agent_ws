import importlib.util
import sys
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[3] / "scripts" / "voice_provider_preflight.py"
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


def test_auto_vad_falls_back_to_energy_when_silero_dependencies_are_missing(tmp_path):
    report = preflight.check_voice_providers(
        mode="offline",
        vad_provider="auto",
        kws_provider="none",
        config_path=write_config(tmp_path / "agent.yaml"),
        module_finder=finder(),
    )

    assert report.ok
    assert report.vad_provider == "energy"
    assert report.mature_vad_active is False
    assert report.vad_maturity == "energy_fallback"
    assert any(item.startswith("vad:auto_fallback:energy:") for item in report.warnings)
    assert "bash scripts/setup_voice_vad_runtime.sh webrtc" in report.recommendations


def test_require_mature_vad_blocks_energy_fallback(tmp_path):
    report = preflight.check_voice_providers(
        mode="offline",
        vad_provider="auto",
        kws_provider="none",
        config_path=write_config(tmp_path / "agent.yaml"),
        require_mature_vad=True,
        module_finder=finder(),
    )

    assert not report.ok
    assert report.vad_provider == "energy"
    assert report.mature_vad_active is False
    assert "vad:mature_provider_required" in report.blockers
    assert "bash scripts/setup_voice_vad_runtime.sh all" in report.recommendations


def test_auto_vad_falls_back_to_webrtc_when_silero_missing_but_webrtc_available(tmp_path):
    report = preflight.check_voice_providers(
        mode="offline",
        vad_provider="auto",
        kws_provider="none",
        config_path=write_config(tmp_path / "agent.yaml"),
        module_finder=finder("webrtcvad"),
    )

    assert report.ok
    assert report.vad_provider == "webrtc"
    assert report.mature_vad_active is True
    assert report.vad_maturity == "mature_acoustic_webrtc"
    assert any(item.startswith("vad:auto_fallback:webrtc:") for item in report.warnings)
    assert "bash scripts/setup_voice_vad_runtime.sh all" in report.recommendations


def test_auto_vad_selects_silero_when_dependencies_are_available(tmp_path):
    model = tmp_path / "silero_vad.onnx"
    model.write_text("stub", encoding="utf-8")
    report = preflight.check_voice_providers(
        mode="offline",
        vad_provider="auto",
        kws_provider="none",
        config_path=write_config(tmp_path / "agent.yaml", silero=str(model)),
        module_finder=finder("onnxruntime"),
    )

    assert report.ok
    assert report.vad_provider == "silero"
    assert report.mature_vad_active is True
    assert report.vad_maturity == "mature_acoustic_silero"
    assert "vad:auto_selected:silero" in report.warnings


def test_webrtc_vad_requires_python_package(tmp_path):
    report = preflight.check_voice_providers(
        mode="online",
        vad_provider="webrtc",
        kws_provider="none",
        config_path=write_config(tmp_path / "agent.yaml"),
        module_finder=finder(),
    )

    assert not report.ok
    assert "vad:webrtcvad_package_missing" in report.blockers
    assert "bash scripts/setup_voice_vad_runtime.sh webrtc" in report.recommendations


def test_silero_onnx_requires_runtime_and_model_path(tmp_path):
    report = preflight.check_voice_providers(
        mode="online",
        vad_provider="silero",
        kws_provider="none",
        config_path=write_config(tmp_path / "agent.yaml"),
        module_finder=finder(),
    )

    assert not report.ok
    assert "vad:onnxruntime_package_missing" in report.blockers
    assert "vad:silero_model_path_empty" in report.blockers
    assert "bash scripts/setup_voice_vad_runtime.sh silero" in report.recommendations


def test_silero_vad_can_be_configured_by_cli_overrides(tmp_path):
    model = tmp_path / "silero_vad.onnx"
    model.write_text("stub", encoding="utf-8")

    report = preflight.check_voice_providers(
        mode="online",
        vad_provider="silero",
        kws_provider="none",
        config_path=write_config(tmp_path / "agent.yaml"),
        vad_overrides={
            "model_path": str(model),
            "use_onnx": "true",
        },
        module_finder=finder("onnxruntime"),
    )

    assert report.ok
    assert report.blockers == ()


def test_silero_vad_cli_override_reports_missing_model_path(tmp_path):
    missing = tmp_path / "missing_silero.onnx"

    report = preflight.check_voice_providers(
        mode="online",
        vad_provider="silero",
        kws_provider="none",
        config_path=write_config(tmp_path / "agent.yaml"),
        vad_overrides={
            "model_path": str(missing),
            "use_onnx": "true",
        },
        module_finder=finder("onnxruntime"),
    )

    assert not report.ok
    assert f"vad:silero_model_missing:{missing}" in report.blockers


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


def test_openwakeword_cli_override_reports_missing_model_path(tmp_path):
    report = preflight.check_voice_providers(
        mode="online",
        vad_provider="energy",
        kws_provider="openwakeword",
        config_path=write_config(tmp_path / "agent.yaml"),
        keyword_overrides={
            "openwakeword_models": str(tmp_path / "custom_wakeword.onnx")
        },
        module_finder=finder("openwakeword.model"),
    )

    assert not report.ok
    assert any(
        item.startswith("kws:openwakeword_model_missing:") for item in report.blockers
    )
    assert "kws:openwakeword_using_default_models" not in report.warnings


def test_openwakeword_missing_parent_package_reports_blocker_without_crashing(tmp_path):
    def raising_finder(module: str):
        if module == "openwakeword.model":
            raise ModuleNotFoundError("No module named 'openwakeword'")
        return None

    report = preflight.check_voice_providers(
        mode="online",
        vad_provider="energy",
        kws_provider="openwakeword",
        config_path=write_config(tmp_path / "agent.yaml"),
        module_finder=raising_finder,
    )

    assert not report.ok
    assert "kws:openwakeword_package_missing" in report.blockers
    assert "bash scripts/setup_voice_kws_runtime.sh openwakeword" in report.recommendations
    assert "kws:openwakeword_using_default_models" not in report.warnings


def test_sherpa_kws_can_be_configured_by_cli_overrides(tmp_path):
    for name in ["tokens.txt", "encoder.onnx", "decoder.onnx", "joiner.onnx", "keywords.txt"]:
        (tmp_path / name).write_text("stub", encoding="utf-8")

    report = preflight.check_voice_providers(
        mode="offline",
        vad_provider="energy",
        kws_provider="sherpa",
        config_path=write_config(tmp_path / "agent.yaml"),
        keyword_overrides={
            "sherpa_tokens": str(tmp_path / "tokens.txt"),
            "sherpa_encoder": str(tmp_path / "encoder.onnx"),
            "sherpa_decoder": str(tmp_path / "decoder.onnx"),
            "sherpa_joiner": str(tmp_path / "joiner.onnx"),
            "sherpa_keywords_file": str(tmp_path / "keywords.txt"),
        },
        module_finder=finder("sherpa_onnx"),
    )

    assert report.ok
    assert report.blockers == ()


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
    assert "vad:onnxruntime_package_missing" in rendered
    assert "vad:silero_model_path_empty" in rendered
    assert "recommendations:" in rendered
    assert "bash scripts/setup_voice_vad_runtime.sh silero" in rendered
