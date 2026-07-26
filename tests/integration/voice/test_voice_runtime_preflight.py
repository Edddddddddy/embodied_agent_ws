import json
import subprocess
import sys
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "voice_runtime_preflight.py"


def test_repository_profiles_pass_contract_only_without_models_or_api_keys():
    for profile in ("offline-edge", "online-cloud"):
        completed = subprocess.run(
            [
                sys.executable,
                str(SCRIPT),
                "--profile",
                profile,
                "--contract-only",
                "--json",
            ],
            cwd=ROOT,
            text=True,
            capture_output=True,
            check=False,
        )

        assert completed.returncode == 0, completed.stderr
        report = json.loads(completed.stdout)
        assert report["profile"] == profile
        assert report["status"] == "contract_valid"
        assert {item["name"] for item in report["components"]} == {
            "vad",
            "asr",
            "llm",
            "rag",
            "tts",
        }


def test_offline_profile_checks_provider_files_not_only_parent_directories():
    profiles = yaml.safe_load(
        (ROOT / "config" / "voice_runtime_profiles.yaml").read_text(
            encoding="utf-8"
        )
    )["profiles"]
    components = profiles["offline-edge"]["components"]
    asr_paths = {
        Path(item["value"]).name
        for item in components["asr"]["candidates"][0]["paths"]
    }
    llm_paths = {
        Path(item["value"]).name
        for item in components["llm"]["candidates"][0]["paths"]
    }
    tts_paths = {
        Path(item["value"]).name
        for item in components["tts"]["candidates"][0]["paths"]
    }

    assert asr_paths == {
        "tokens.txt",
        "encoder-epoch-99-avg-1.int8.onnx",
        "decoder-epoch-99-avg-1.int8.onnx",
        "joiner-epoch-99-avg-1.int8.onnx",
    }
    assert llm_paths == {"llama-server", "Qwen3-0.6B-Q8_0.gguf"}
    assert components["llm"]["candidates"][0]["paths"][0]["kind"] == "executable"
    assert tts_paths == {"model.int8.onnx", "tokens.txt", "lexicon.txt"}
    assert len(components["tts"]["candidates"]) == 1


def test_cli_can_validate_a_ready_fixture_profile(tmp_path):
    config = tmp_path / "profiles.yaml"
    config.write_text(
        """
profiles:
  offline-edge:
    components:
      vad:
        required: true
        candidates:
          - provider: energy
      asr:
        required: true
        candidates:
          - provider: fixture_asr
      llm:
        required: true
        candidates:
          - provider: fixture_llm
      rag:
        required: true
        candidates:
          - provider: fixture_rag
      tts:
        required: true
        candidates:
          - provider: fixture_tts
""".strip(),
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--profile",
            "offline-edge",
            "--config",
            str(config),
            "--workspace",
            str(tmp_path),
            "--json",
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr
    report = json.loads(completed.stdout)
    assert report["status"] == "ready"
    assert report["blockers"] == []


def test_acceptance_wrapper_rejects_missing_profile_argument():
    completed = subprocess.run(
        ["bash", "scripts/acceptance_test.sh", "voice-runtime-preflight"],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )

    assert completed.returncode == 2
    assert "voice-runtime-preflight {offline-edge|online-cloud}" in completed.stderr
