import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")


def test_demo_evidence_checklist_passes_with_synthetic_reports(tmp_path):
    automatic = tmp_path / "demo_acceptance_report.json"
    voice = tmp_path / "continuous-live-check.json"
    nav2 = tmp_path / "nav2-live-check.json"
    recording = tmp_path / "demo_recording.mp4"
    screenshot = tmp_path / "demo_screenshot.png"
    output = tmp_path / "checklist.json"
    markdown = tmp_path / "checklist.md"

    _write_json(
        automatic,
        {
            "scenario": "job_showcase_release_gate",
            "profile": "demo",
            "ok": True,
            "command_count": 5,
            "evidence_summary": {"mock_ros": 2},
            "evidence_policy": {"manual_followups": ["continuous-offline"]},
        },
    )
    _write_json(
        voice,
        {
            "ok": True,
            "asr_count": 7,
            "action_candidate_count": 5,
            "action_success_count": 5,
            "final_cmd_vel_zero": True,
            "missing": [],
        },
    )
    _write_json(
        nav2,
        {
            "ok": True,
            "action_candidate_names": {"navigate_to": 1, "follow_waypoints": 1},
            "navigation_failure_reasons": [],
        },
    )
    recording.write_bytes(b"demo")
    screenshot.write_bytes(b"png")

    completed = subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts" / "demo_evidence_checklist.py"),
            "--workspace",
            str(ROOT),
            "--automatic-report",
            str(automatic),
            "--voice-report",
            str(voice),
            "--nav2-report",
            str(nav2),
            "--recording",
            str(recording),
            "--screenshot",
            str(screenshot),
            "--output",
            str(output),
            "--markdown",
            str(markdown),
            "--require-nav2",
            "--require-visual-evidence",
            "--strict",
        ],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )

    assert completed.returncode == 0, completed.stdout
    report = json.loads(output.read_text(encoding="utf-8"))
    assert report["ok"] is True
    assert report["required_passed"] == report["required_total"]
    assert "demo_recording" not in report["missing_required"]
    assert "求职展示演示证据 Checklist" in markdown.read_text(encoding="utf-8")


def test_demo_evidence_checklist_non_strict_writes_missing_actions(tmp_path):
    output = tmp_path / "checklist.json"
    markdown = tmp_path / "checklist.md"
    completed = subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts" / "demo_evidence_checklist.py"),
            "--workspace",
            str(ROOT),
            "--automatic-report",
            str(tmp_path / "missing_auto.json"),
            "--voice-report",
            str(tmp_path / "missing_voice.json"),
            "--output",
            str(output),
            "--markdown",
            str(markdown),
        ],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )

    assert completed.returncode == 0, completed.stdout
    report = json.loads(output.read_text(encoding="utf-8"))
    assert report["ok"] is False
    assert "automatic_demo_gate" in report["missing_required"]
    assert "continuous_voice_live" in report["missing_required"]
    assert "先运行 demo-gate" in markdown.read_text(encoding="utf-8")
