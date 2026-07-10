import json
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_showcase_release_gate_dry_run_writes_report(tmp_path):
    output = tmp_path / "acceptance_report.json"
    completed = subprocess.run(
        [
            "python3",
            str(ROOT / "scripts" / "showcase_release_gate.py"),
            "--workspace",
            str(ROOT),
            "--dry-run",
            "--output",
            str(output),
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    assert completed.returncode == 0, completed.stdout
    report = json.loads(output.read_text(encoding="utf-8"))
    assert report["scenario"] == "job_showcase_release_gate"
    assert report["profile"] == "core"
    assert report["dry_run"] is True
    assert report["ok"] is True
    assert report["command_count"] == 5
    command_names = {item["name"] for item in report["commands"]}
    assert {
        "python_repository_and_agent_units",
        "cli_and_instruction_parser",
        "continuous_voice_queue",
        "voice_navigation_demo",
        "offline_runtime_and_cpp_ros",
    } == command_names
    assert report["evidence_policy"]["profile"] == "core"
    assert "continuous-offline" in report["evidence_policy"]["manual_followups"]
    by_name = {item["name"]: item for item in report["commands"]}
    assert by_name["continuous_voice_queue"]["evidence_kind"] == "mock_ros"
    assert by_name["offline_runtime_and_cpp_ros"]["evidence_kind"] == "local_runtime"
    assert report["evidence_summary"]["mock_ros"] >= 1
    assert report["evidence_summary"]["local_runtime"] >= 1


def test_showcase_release_gate_full_profile_keeps_expanded_checks(tmp_path):
    output = tmp_path / "acceptance_report_full.json"
    completed = subprocess.run(
        [
            "python3",
            str(ROOT / "scripts" / "showcase_release_gate.py"),
            "--workspace",
            str(ROOT),
            "--dry-run",
            "--profile",
            "full",
            "--output",
            str(output),
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    assert completed.returncode == 0, completed.stdout
    report = json.loads(output.read_text(encoding="utf-8"))
    assert report["profile"] == "full"
    assert report["command_count"] >= 9
    command_names = {item["name"] for item in report["commands"]}
    assert {
        "repository_and_offline_unit",
        "instruction_parser_eval",
        "continuous_multi_command",
        "navigation_demo",
        "offline_latency",
        "summer_tts_service",
        "cpp_ros_unit",
    } <= command_names
    assert report["evidence_policy"]["profile"] == "full"
    assert "nav2-turtlebot3" in report["evidence_policy"]["manual_followups"]


def test_showcase_release_gate_demo_profile_targets_pre_demo_evidence(tmp_path):
    output = tmp_path / "demo_acceptance_report.json"
    completed = subprocess.run(
        [
            "python3",
            str(ROOT / "scripts" / "showcase_release_gate.py"),
            "--workspace",
            str(ROOT),
            "--dry-run",
            "--profile",
            "demo",
            "--output",
            str(output),
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    assert completed.returncode == 0, completed.stdout
    report = json.loads(output.read_text(encoding="utf-8"))
    assert report["profile"] == "demo"
    assert report["command_count"] == 5
    command_names = {item["name"] for item in report["commands"]}
    assert {
        "demo_cli_readiness",
        "voice_provider_readiness",
        "speaker_memory_preferences",
        "continuous_voice_demo",
        "navigation_and_offline_evidence",
    } == command_names
    commands = "\n".join(item["command"] for item in report["commands"])
    assert "provider-preflight" in commands
    assert "voice-calibration-report" in commands
    assert "speaker-memory-mock" in commands
    assert "offline-showcase-report" in commands
    assert report["evidence_policy"]["profile"] == "demo"
    assert report["evidence_policy"]["requires_human_demo"] is True
    assert "continuous-offline" in report["evidence_policy"]["manual_followups"]
    by_name = {item["name"]: item for item in report["commands"]}
    assert by_name["voice_provider_readiness"]["evidence_kind"] == "local_preflight"
    assert by_name["continuous_voice_demo"]["evidence_kind"] == "mock_ros"
