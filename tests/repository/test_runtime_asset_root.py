"""代码 worktree 与未跟踪运行时资产根目录的 shell 契约测试。"""

from __future__ import annotations

import os
from pathlib import Path
import shlex
import shutil
import subprocess
import time


ROOT = Path(__file__).resolve().parents[2]
LIFECYCLE_UTILS = ROOT / "scripts" / "lifecycle_utils.sh"


def test_core_gate_uses_current_code_but_shared_runtime_venv():
    script = (ROOT / "scripts" / "run_core_tests.sh").read_text(
        encoding="utf-8"
    )

    assert 'embodied_resolve_workspace "${BASH_SOURCE[0]}"' in script
    assert "embodied_resolve_runtime_root" in script
    assert '$EMBODIED_RUNTIME_ROOT/.venv/bin/python' in script
    assert '"$PYTHON_BIN" -m pytest' in script
    assert "/home/ubuntu/embodied_agent_ws" not in script


def test_offline_typed_gate_uses_current_code_and_shared_model_root():
    smoke = (
        ROOT / "scripts/smoke_test_offline_sherpa_typed_simulation.sh"
    ).read_text(encoding="utf-8")
    probe = (
        ROOT
        / "tools/acceptance/probes/voice/offline_sherpa_typed_simulation.py"
    ).read_text(encoding="utf-8")

    assert 'embodied_resolve_workspace "${BASH_SOURCE[0]}"' in smoke
    assert "embodied_resolve_runtime_root" in smoke
    assert "EMBODIED_RUNTIME_ROOT" in probe
    assert "/home/ubuntu/embodied_agent_ws/models" not in probe


def test_offline_agent_deployment_paths_follow_runtime_root():
    launch = (
        ROOT / "src/embodied_offline_agent/launch/offline_agent.launch.py"
    ).read_text(encoding="utf-8")
    config = (
        ROOT / "src/embodied_offline_agent/config/offline_agent.yaml"
    ).read_text(encoding="utf-8")
    parameters = (
        ROOT
        / "src/embodied_agent_core/embodied_agent_core/agent_parameters.py"
    ).read_text(encoding="utf-8")

    assert "EMBODIED_RUNTIME_ROOT" in launch
    assert 'DeclareLaunchArgument("asr_model_dir"' in launch
    assert 'DeclareLaunchArgument("tts_model_dir"' in launch
    assert '"asr_model_dir": asr_model_dir' in launch
    assert '"tts_model_dir": tts_model_dir' in launch
    assert "EMBODIED_RUNTIME_ROOT" in parameters
    assert "/home/ubuntu/embodied_agent_ws" not in launch
    assert "/home/ubuntu/embodied_agent_ws" not in config
    assert "/home/ubuntu/embodied_agent_ws" not in parameters


def _clean_environment() -> dict[str, str]:
    environment = os.environ.copy()
    for name in (
        "AMENT_PREFIX_PATH",
        "COLCON_PREFIX_PATH",
        "EMBODIED_RUNTIME_ROOT",
        "EMBODIED_ROS_SETUP",
        "LLAMA_MODEL",
        "LLAMA_SERVER",
        "PYTHONPATH",
        "VIRTUAL_ENV",
        "VOICE_CALIBRATION_ENV",
        "WORKSPACE",
    ):
        environment.pop(name, None)
    return environment


def _run_bash(script: str, *, environment: dict[str, str] | None = None):
    return subprocess.run(
        ["bash", "--noprofile", "--norc", "-c", script],
        cwd=ROOT,
        env=environment or _clean_environment(),
        capture_output=True,
        text=True,
        check=False,
    )


def _make_linked_worktree(tmp_path: Path) -> tuple[Path, Path]:
    main = tmp_path / "main"
    linked = tmp_path / "feature"
    subprocess.run(["git", "init", "-q", str(main)], check=True)
    (main / "README.md").write_text("fixture\n", encoding="utf-8")
    subprocess.run(["git", "-C", str(main), "add", "README.md"], check=True)
    subprocess.run(
        [
            "git",
            "-C",
            str(main),
            "-c",
            "user.name=Runtime Fixture",
            "-c",
            "user.email=runtime@example.invalid",
            "commit",
            "-qm",
            "fixture",
        ],
        check=True,
    )
    subprocess.run(
        [
            "git",
            "-C",
            str(main),
            "worktree",
            "add",
            "-q",
            "-b",
            "feature",
            str(linked),
        ],
        check=True,
    )
    return main.resolve(), linked.resolve()


def test_runtime_root_prefers_explicit_override(tmp_path: Path):
    main, linked = _make_linked_worktree(tmp_path)
    explicit = tmp_path / "shared-runtime"
    explicit.mkdir()
    script = f"""
set -euo pipefail
export WORKSPACE={shlex.quote(str(linked))}
export EMBODIED_RUNTIME_ROOT={shlex.quote(str(explicit))}
source {shlex.quote(str(LIFECYCLE_UTILS))}
embodied_resolve_runtime_root
printf '%s\n' "$EMBODIED_RUNTIME_ROOT"
"""

    completed = _run_bash(script)

    assert completed.returncode == 0, completed.stderr
    assert completed.stdout.strip() == str(explicit.resolve())
    assert completed.stdout.strip() != str(main)


def test_runtime_root_uses_git_common_dir_main_worktree(tmp_path: Path):
    main, linked = _make_linked_worktree(tmp_path)
    script = f"""
set -euo pipefail
export WORKSPACE={shlex.quote(str(linked))}
unset EMBODIED_RUNTIME_ROOT
source {shlex.quote(str(LIFECYCLE_UTILS))}
embodied_resolve_runtime_root
printf '%s\n' "$EMBODIED_RUNTIME_ROOT"
"""

    completed = _run_bash(script)

    assert completed.returncode == 0, completed.stderr
    assert completed.stdout.strip() == str(main)


def test_runtime_root_falls_back_to_workspace_outside_git(tmp_path: Path):
    workspace = tmp_path / "plain-workspace"
    workspace.mkdir()
    script = f"""
set -euo pipefail
export WORKSPACE={shlex.quote(str(workspace))}
unset EMBODIED_RUNTIME_ROOT
source {shlex.quote(str(LIFECYCLE_UTILS))}
embodied_resolve_runtime_root
printf '%s\n' "$EMBODIED_RUNTIME_ROOT"
"""

    completed = _run_bash(script)

    assert completed.returncode == 0, completed.stderr
    assert completed.stdout.strip() == str(workspace.resolve())


def _make_continuous_fixture(tmp_path: Path) -> tuple[Path, Path, Path]:
    workspace = tmp_path / "code-worktree"
    scripts = workspace / "scripts"
    scripts.mkdir(parents=True)
    for name in (
        "activate.sh",
        "continuous_nav2_voice_control.sh",
        "lifecycle_utils.sh",
        "start_llama_server.sh",
        "voice_control_profile.sh",
    ):
        shutil.copy2(ROOT / "scripts" / name, scripts / name)

    ros_setup = tmp_path / "fake-ros" / "setup.bash"
    ros_setup.parent.mkdir(parents=True)
    ros_setup.write_text("export FAKE_ROS_SOURCED=1\n", encoding="utf-8")
    install_setup = workspace / "install" / "setup.bash"
    install_setup.parent.mkdir(parents=True)
    install_setup.write_text("export FAKE_INSTALL_SOURCED=1\n", encoding="utf-8")

    runtime_root = tmp_path / "shared-runtime"
    (runtime_root / "logs").mkdir(parents=True)
    (runtime_root / "models" / "silero_vad").mkdir(parents=True)
    (runtime_root / "third_party" / "llama.cpp" / "build" / "bin").mkdir(
        parents=True
    )
    return workspace, runtime_root, ros_setup


def _continuous_environment(
    workspace: Path, runtime_root: Path, ros_setup: Path
) -> dict[str, str]:
    environment = _clean_environment()
    environment.update(
        {
            "APPLY_VOICE_CALIBRATION": "false",
            "CONTINUOUS_MONITOR_ENABLED": "false",
            "CONTINUOUS_PREFLIGHT_ENABLED": "false",
            "CONTINUOUS_READINESS_ENABLED": "false",
            "EMBODIED_ROS_SETUP": str(ros_setup),
            "EMBODIED_RUNTIME_ROOT": str(runtime_root),
            "NAV2_MICROPHONE_ENABLED": "false",
            "PULSE_CAPTURE_BRIDGE": "false",
            "VAD_PROVIDER": "energy",
            "WORKSPACE": str(workspace),
        }
    )
    return environment


def test_continuous_nav2_resolves_all_runtime_assets_from_shared_root(
    tmp_path: Path,
):
    workspace, runtime_root, ros_setup = _make_continuous_fixture(tmp_path)
    environment = _continuous_environment(workspace, runtime_root, ros_setup)
    environment["CONTINUOUS_PRINT_CONFIG"] = "true"

    completed = subprocess.run(
        [
            "bash",
            str(workspace / "scripts" / "continuous_nav2_voice_control.sh"),
            "offline",
        ],
        cwd=workspace,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr
    assert f"EMBODIED_RUNTIME_ROOT={runtime_root}" in completed.stdout
    assert (
        f"LLAMA_SERVER={runtime_root}/third_party/llama.cpp/build/bin/llama-server"
        in completed.stdout
    )
    assert f"LLAMA_MODEL={runtime_root}/models/Qwen3-0.6B-Q8_0.gguf" in completed.stdout
    assert (
        f"SILERO_VAD_MODEL_PATH={runtime_root}/models/silero_vad/silero_vad.onnx"
        in completed.stdout
    )
    assert f"VOICE_CALIBRATION_ENV={runtime_root}/logs/voice_calibration.env" in completed.stdout
    assert f"LLAMA_SERVER={workspace}/third_party" not in completed.stdout


def test_continuous_nav2_fails_immediately_when_llama_process_exits(
    tmp_path: Path,
):
    workspace, runtime_root, ros_setup = _make_continuous_fixture(tmp_path)
    server = runtime_root / "third_party" / "llama.cpp" / "build" / "bin" / "llama-server"
    server.write_text("#!/usr/bin/env bash\nexit 23\n", encoding="utf-8")
    server.chmod(0o755)
    (runtime_root / "models" / "Qwen3-0.6B-Q8_0.gguf").write_text(
        "fixture\n", encoding="utf-8"
    )
    fake_bin = tmp_path / "fake-bin"
    fake_bin.mkdir()
    fake_curl = fake_bin / "curl"
    fake_curl.write_text("#!/usr/bin/env bash\nexit 22\n", encoding="utf-8")
    fake_curl.chmod(0o755)

    environment = _continuous_environment(workspace, runtime_root, ros_setup)
    environment["PATH"] = f"{fake_bin}:{environment['PATH']}"
    started = time.monotonic()
    completed = subprocess.run(
        [
            "timeout",
            "5",
            "bash",
            str(workspace / "scripts" / "continuous_nav2_voice_control.sh"),
            "offline",
        ],
        cwd=workspace,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )
    elapsed_s = time.monotonic() - started

    assert completed.returncode == 1, completed.stderr
    assert elapsed_s < 3.0
    assert "llama.cpp server 进程提前退出" in completed.stderr
    assert "exit=23" in completed.stderr
