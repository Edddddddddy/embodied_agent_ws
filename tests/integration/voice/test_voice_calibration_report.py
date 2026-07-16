import importlib.util
import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "voice_calibration_report.py"
spec = importlib.util.spec_from_file_location("voice_calibration_report", SCRIPT)
voice_report = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = voice_report
spec.loader.exec_module(voice_report)


def test_low_gain_bundle_recommends_copyable_environment():
    report = voice_report.build_calibration_report(
        mode="offline",
        audio_samples=voice_report._synthetic_audio_samples("low_gain"),
        kws_samples=[],
        require_kws=False,
        vad_provider="energy",
        kws_provider="none",
    )

    assert report["ok"] is True
    assert report["decision"] == "needs_tuning"
    assert "VOICE_CONTROL_PROFILE=low_gain" in report["recommended_environment"]
    assert any(
        item.startswith("SPEECH_START_THRESHOLD=")
        for item in report["recommended_environment"]
    )
    assert "continuous-offline" in report["next_command"]
    assert "audio:microphone_low_gain" in report["warnings"]
    env_text = voice_report.render_env(report)
    assert "export VOICE_CONTROL_PROFILE=low_gain" in env_text
    assert "export SPEECH_START_THRESHOLD=" in env_text


def test_auto_vad_energy_fallback_includes_provider_setup_advice():
    report = voice_report.build_calibration_report(
        mode="offline",
        audio_samples=voice_report._synthetic_audio_samples("ready"),
        kws_samples=[],
        require_kws=False,
        vad_provider="auto",
        kws_provider="none",
        module_finder=lambda _module: None,
    )

    assert report["provider"]["vad_provider"] == "energy"
    assert "VAD_PROVIDER=energy" in report["recommended_environment"]
    assert "bash scripts/setup_voice_vad_runtime.sh webrtc" in report[
        "provider_setup_commands"
    ]
    assert any(
        item.startswith("provider:vad:auto_fallback:energy:")
        for item in report["warnings"]
    )
    markdown = voice_report.render_markdown(report)
    assert "Provider setup" in markdown
    assert "setup_voice_vad_runtime.sh webrtc" in markdown


def test_kws_samples_add_copyable_threshold_environment():
    report = voice_report.build_calibration_report(
        mode="offline",
        audio_samples=voice_report._synthetic_audio_samples("ready"),
        kws_samples=voice_report._synthetic_kws_samples("ready"),
        require_kws=True,
        vad_provider="webrtc",
        kws_provider="openwakeword",
    )

    threshold_items = [
        item for item in report["recommended_environment"]
        if item.startswith("OPENWAKEWORD_THRESHOLD=")
    ]
    assert threshold_items
    assert "KWS_PROVIDER=openwakeword" in report["recommended_environment"]
    assert "OPENWAKEWORD_THRESHOLD=" in report["next_command"]
    env_text = voice_report.render_env(report)
    assert "export OPENWAKEWORD_THRESHOLD=" in env_text


def test_livekit_kws_samples_add_copyable_threshold_environment():
    report = voice_report.build_calibration_report(
        mode="online",
        audio_samples=voice_report._synthetic_audio_samples("ready"),
        kws_samples=voice_report._synthetic_kws_samples("ready"),
        require_kws=True,
        vad_provider="webrtc",
        kws_provider="livekit",
    )

    assert any(
        item.startswith("LIVEKIT_WAKEWORD_THRESHOLD=")
        for item in report["recommended_environment"]
    )
    assert "LIVEKIT_WAKEWORD_THRESHOLD=" in report["next_command"]


def test_voice_calibration_report_cli_writes_json_and_markdown(tmp_path):
    json_output = tmp_path / "voice_calibration_report.json"
    md_output = tmp_path / "voice_calibration_report.md"
    env_output = tmp_path / "voice_calibration.env"
    completed = subprocess.run(
        [
            "python3",
            str(SCRIPT),
            "--synthetic-profile",
            "ready",
            "--vad-provider",
            "energy",
            "--json-output",
            str(json_output),
            "--md-output",
            str(md_output),
            "--env-output",
            str(env_output),
        ],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )

    assert completed.returncode == 0, completed.stdout
    payload = json.loads(json_output.read_text(encoding="utf-8"))
    assert payload["scenario"] == "voice_calibration_report"
    assert payload["decision"] == "ready"
    assert payload["audio"]["recommended_voice_profile"] == "normal"
    markdown = md_output.read_text(encoding="utf-8")
    assert "真实语音控制校准报告" in markdown
    assert "下一条建议命令" in markdown
    assert "source logs/voice_calibration.env" in markdown
    env_text = env_output.read_text(encoding="utf-8")
    assert "export VOICE_CONTROL_PROFILE=normal" in env_text
    assert "export VAD_PROVIDER=energy" in env_text
