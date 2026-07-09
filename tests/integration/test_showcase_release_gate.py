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
    assert report["dry_run"] is True
    assert report["ok"] is True
    command_names = {item["name"] for item in report["commands"]}
    assert {
        "repository_and_offline_unit",
        "continuous_multi_command",
        "navigation_demo",
        "offline_latency",
        "summer_tts_service",
        "cpp_ros_unit",
    } <= command_names
