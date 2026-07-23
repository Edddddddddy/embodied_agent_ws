#!/usr/bin/env python3
"""Runtime contract tests for microphone readiness fail-fast behavior."""

from __future__ import annotations

import os
import stat
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def _write_executable(path: Path, content: str) -> None:
    path.write_text(content, encoding="utf-8")
    path.chmod(path.stat().st_mode | stat.S_IXUSR)


def _runtime_harness(tmp_path: Path) -> tuple[Path, dict[str, str]]:
    """Create public-command fakes without mocking launcher internals."""

    workspace = tmp_path / "workspace"
    scripts = workspace / "scripts"
    fake_bin = tmp_path / "bin"
    scripts.mkdir(parents=True)
    fake_bin.mkdir()

    # Launcher-owned shell contracts stay real; only heavyweight ROS/audio
    # processes are replaced by deterministic public command fakes.
    (scripts / "voice_control_profile.sh").symlink_to(
        ROOT / "scripts" / "voice_control_profile.sh"
    )
    _write_executable(scripts / "activate.sh", "#!/usr/bin/env bash\nreturn 0\n")
    _write_executable(
        scripts / "cleanup_simulation_processes.sh",
        "#!/usr/bin/env bash\nexit 0\n",
    )
    for name in (
        "pulse_audio_capture_bridge.py",
        "system_readiness_check.py",
        "simulation_readiness_check.py",
        "voice_control_readiness_check.py",
        "continuous_voice_monitor.py",
        "publish_nav2_initial_pose.py",
    ):
        (scripts / name).touch()

    _write_executable(
        fake_bin / "pactl",
        "#!/usr/bin/env bash\nprintf '2\\tRDPSource\\n'\n",
    )
    _write_executable(
        fake_bin / "parecord",
        "#!/usr/bin/env bash\nexit 0\n",
    )
    _write_executable(
        fake_bin / "ros2",
        "#!/usr/bin/env bash\ntrap 'exit 0' INT TERM\nsleep 1\n",
    )
    _write_executable(
        fake_bin / "python3",
        """#!/usr/bin/env bash
case "$1" in
  */pulse_audio_capture_bridge.py)
    # Stay alive long enough to pass the launcher's bridge startup probe, then
    # exit on our own so a deliberately non-fail-fast implementation cannot
    # leave the contract test hanging forever.
    sleep 2
    exit 0
    ;;
  */system_readiness_check.py|*/simulation_readiness_check.py)
    exit 0
    ;;
  */voice_control_readiness_check.py)
    echo 'FAKE: microphone readiness failed' >&2
    exit 1
    ;;
  */continuous_voice_monitor.py)
    exit 0
    ;;
  *)
    echo "unexpected python command: $*" >&2
    exit 91
    ;;
esac
""",
    )

    env = os.environ.copy()
    env.update(
        {
            "WORKSPACE": str(workspace),
            "PATH": f"{fake_bin}:{env['PATH']}",
            "PULSE_CAPTURE_BRIDGE": "true",
            "PULSE_CAPTURE_SOURCE": "RDPSource",
            "PULSE_SERVER": "unix:/mnt/wslg/PulseServer",
            "VAD_PROVIDER": "energy",
            "CONTINUOUS_PREFLIGHT_ENABLED": "false",
            "CONTINUOUS_MONITOR_ENABLED": "false",
            "CONTINUOUS_READINESS_ENABLED": "true",
            "CONTINUOUS_READINESS_DURATION": "0.1",
            "SIMULATION_READINESS_ENABLED": "false",
            "APPLY_VOICE_CALIBRATION": "false",
        }
    )
    env.pop("CONTINUOUS_READINESS_REQUIRED", None)
    return workspace, env


def test_continuous_voice_real_pulse_capture_fails_fast_when_readiness_fails(
    tmp_path: Path,
) -> None:
    _, env = _runtime_harness(tmp_path)

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "continuous_voice_control.sh"), "online"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=15,
        check=False,
    )

    assert result.returncode != 0
    assert "CONTINUOUS_READINESS_REQUIRED=true" in result.stdout
    assert "请在接下来的 0.1s 采样窗口内持续说完整话" in result.stdout
    assert "真实麦克风 readiness 未通过" in result.stderr
    assert "请现在对着麦克风" in result.stderr


def test_continuous_voice_can_explicitly_make_readiness_non_blocking(
    tmp_path: Path,
) -> None:
    _, env = _runtime_harness(tmp_path)
    env["CONTINUOUS_READINESS_REQUIRED"] = "false"

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "continuous_voice_control.sh"), "online"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=15,
        check=False,
    )

    assert result.returncode == 0
    assert "CONTINUOUS_READINESS_REQUIRED=false" in result.stdout
    assert "WARN: readiness check 未完全通过；仍继续运行" in result.stderr


def test_nav2_voice_real_pulse_capture_fails_fast_when_readiness_fails(
    tmp_path: Path,
) -> None:
    _, env = _runtime_harness(tmp_path)
    env.update(
        {
            "NAV2_MICROPHONE_ENABLED": "true",
            "NAV2_CAPTURE_ENABLED": "true",
            "NAV2_PROVIDER_MODE": "online",
            "NAV2_SLAM": "true",
        }
    )

    result = subprocess.run(
        [
            "bash",
            str(ROOT / "scripts" / "continuous_nav2_voice_control.sh"),
            "online",
        ],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=15,
        check=False,
    )

    assert result.returncode != 0
    assert "CONTINUOUS_READINESS_REQUIRED=true" in result.stdout
    assert "请在接下来的 0.1s 采样窗口内持续说完整话" in result.stdout
    assert "真实麦克风 readiness 未通过" in result.stderr
    assert "请现在对着麦克风" in result.stderr


def test_nav2_mock_without_microphone_can_continue_after_readiness_failure(
    tmp_path: Path,
) -> None:
    _, env = _runtime_harness(tmp_path)
    env.update(
        {
            "NAV2_MICROPHONE_ENABLED": "false",
            "NAV2_CAPTURE_ENABLED": "false",
            "NAV2_PROVIDER_MODE": "mock",
            "NAV2_SLAM": "true",
            "PULSE_CAPTURE_BRIDGE": "auto",
        }
    )

    result = subprocess.run(
        [
            "bash",
            str(ROOT / "scripts" / "continuous_nav2_voice_control.sh"),
            "online",
        ],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=15,
        check=False,
    )

    assert result.returncode == 0
    assert "PULSE_CAPTURE_BRIDGE=auto（active=false" in result.stdout
    assert "CONTINUOUS_READINESS_REQUIRED=false" in result.stdout
    assert "WARN: readiness check 未完全通过；仍继续运行" in result.stderr


def test_continuous_voice_defaults_to_four_second_required_readiness(
    tmp_path: Path,
) -> None:
    _, env = _runtime_harness(tmp_path)
    env["CONTINUOUS_PRINT_CONFIG"] = "true"
    env.pop("CONTINUOUS_READINESS_DURATION", None)

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "continuous_voice_control.sh"), "online"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=15,
        check=True,
    )

    assert "CONTINUOUS_READINESS_DURATION=4.0" in result.stdout
    assert "CONTINUOUS_READINESS_REQUIRED=true" in result.stdout


def test_real_voice_launchers_pass_required_speech_to_readiness_probe():
    voice = (ROOT / "scripts" / "continuous_voice_control.sh").read_text(
        encoding="utf-8"
    )
    nav2 = (ROOT / "scripts" / "continuous_nav2_voice_control.sh").read_text(
        encoding="utf-8"
    )

    for content in (voice, nav2):
        assert 'readiness_args+=(--require-speech)' in content
        assert '"${readiness_args[@]}"' in content


def test_nav2_voice_defaults_to_four_second_required_readiness(
    tmp_path: Path,
) -> None:
    _, env = _runtime_harness(tmp_path)
    env.update(
        {
            "CONTINUOUS_PRINT_CONFIG": "true",
            "NAV2_MICROPHONE_ENABLED": "true",
            "NAV2_PROVIDER_MODE": "online",
        }
    )
    env.pop("CONTINUOUS_READINESS_DURATION", None)

    result = subprocess.run(
        [
            "bash",
            str(ROOT / "scripts" / "continuous_nav2_voice_control.sh"),
            "online",
        ],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=15,
        check=True,
    )

    assert "CONTINUOUS_READINESS_DURATION=4.0" in result.stdout
    assert "CONTINUOUS_READINESS_REQUIRED=true" in result.stdout
