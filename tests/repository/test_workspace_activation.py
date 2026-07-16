"""工作区解析与 shell 激活契约测试。"""

from __future__ import annotations

import os
from pathlib import Path
import shlex
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[2]
ACTIVATE = ROOT / "scripts" / "activate.sh"
WORKSPACE_HELPER = ROOT / "scripts" / "lifecycle_utils.sh"


def _run_bash(script: str) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    # 开发者可能已在另一个 ROS/worktree shell 中运行 pytest；契约测试必须与外部叠层隔离。
    for name in (
        "AMENT_PREFIX_PATH",
        "COLCON_PREFIX_PATH",
        "EMBODIED_ROS_SETUP",
        "PYTHONPATH",
        "VIRTUAL_ENV",
        "WORKSPACE",
    ):
        env.pop(name, None)
    return subprocess.run(
        ["bash", "--noprofile", "--norc", "-c", script],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )


def _make_activation_fixture(tmp_path: Path, *, with_install: bool = True) -> tuple[Path, Path]:
    workspace = tmp_path / "renamed-worktree"
    (workspace / "scripts").mkdir(parents=True)
    shutil.copy2(ACTIVATE, workspace / "scripts" / "activate.sh")
    shutil.copy2(WORKSPACE_HELPER, workspace / "scripts" / "lifecycle_utils.sh")

    ros_setup = tmp_path / "fake-ros" / "setup.bash"
    ros_setup.parent.mkdir(parents=True)
    ros_setup.write_text("export FAKE_ROS_SOURCED=1\n", encoding="utf-8")

    if with_install:
        install_setup = workspace / "install" / "setup.bash"
        install_setup.parent.mkdir(parents=True)
        install_setup.write_text("export FAKE_INSTALL_SOURCED=1\n", encoding="utf-8")
    (workspace / ".env").write_text("DEMO_FROM_ENV=loaded\n", encoding="utf-8")
    return workspace, ros_setup


def test_workspace_helper_derives_root_from_calling_script_and_exports_it(tmp_path: Path):
    workspace = tmp_path / "arbitrary-name"
    caller = workspace / "scripts" / "demo.sh"
    caller.parent.mkdir(parents=True)
    caller.touch()
    script = f"""
unset WORKSPACE
source {shlex.quote(str(WORKSPACE_HELPER))}
embodied_resolve_workspace {shlex.quote(str(caller))}
printf 'resolved=%s\n' "$WORKSPACE"
bash -c 'printf "child=%s\\n" "$WORKSPACE"'
"""

    completed = _run_bash(script)

    assert completed.returncode == 0, completed.stderr
    assert f"resolved={workspace.resolve()}" in completed.stdout
    assert f"child={workspace.resolve()}" in completed.stdout


def test_workspace_helper_honors_explicit_exported_override(tmp_path: Path):
    inferred = tmp_path / "inferred"
    override = tmp_path / "explicit"
    (inferred / "scripts").mkdir(parents=True)
    override.mkdir()
    script = f"""
export WORKSPACE={shlex.quote(str(override))}
export EMBODIED_ALLOW_WORKSPACE_OVERRIDE=true
source {shlex.quote(str(WORKSPACE_HELPER))}
embodied_resolve_workspace {shlex.quote(str(inferred / 'scripts' / 'demo.sh'))}
printf '%s\n' "$WORKSPACE"
"""

    completed = _run_bash(script)

    assert completed.returncode == 0, completed.stderr
    assert completed.stdout.strip() == str(override.resolve())


def test_workspace_helper_rejects_stale_override_from_another_worktree(tmp_path: Path):
    inferred = tmp_path / "feature"
    stale = tmp_path / "main"
    caller = inferred / "scripts" / "activate.sh"
    caller.parent.mkdir(parents=True)
    caller.touch()
    stale.mkdir()
    script = f"""
export WORKSPACE={shlex.quote(str(stale))}
source {shlex.quote(str(WORKSPACE_HELPER))}
embodied_resolve_workspace {shlex.quote(str(caller))}
"""

    completed = _run_bash(script)

    assert completed.returncode != 0
    assert "WORKSPACE 与当前入口所属仓库不一致" in completed.stderr
    assert "unset WORKSPACE" in completed.stderr


@pytest.mark.parametrize(
    "enabled_options",
    [(), ("e",), ("u",), ("a",), ("e", "u", "a")],
)
def test_activate_derives_workspace_and_preserves_shell_options(tmp_path: Path, enabled_options):
    workspace, ros_setup = _make_activation_fixture(tmp_path)
    option_commands = "\n".join(f"set -{option}" for option in enabled_options)
    expected = "".join(option for option in "eua" if option in enabled_options)
    script = f"""
unset WORKSPACE VIRTUAL_ENV
set +e +u +a
{option_commands}
export EMBODIED_ROS_SETUP={shlex.quote(str(ros_setup))}
if source {shlex.quote(str(workspace / 'scripts' / 'activate.sh'))}; then
  :
else
  printf 'activation failed unexpectedly\n' >&2
  exit 90
fi
actual=''
case "$-" in *e*) actual="${{actual}}e";; esac
case "$-" in *u*) actual="${{actual}}u";; esac
case "$-" in *a*) actual="${{actual}}a";; esac
printf 'options=%s\n' "$actual"
printf 'workspace=%s\n' "$WORKSPACE"
printf 'ros=%s install=%s env=%s\n' "$FAKE_ROS_SOURCED" "$FAKE_INSTALL_SOURCED" "$DEMO_FROM_ENV"
bash -c 'printf "child_workspace=%s child_env=%s\\n" "$WORKSPACE" "$DEMO_FROM_ENV"'
"""

    completed = _run_bash(script)

    assert completed.returncode == 0, completed.stderr
    assert f"options={expected}" in completed.stdout
    assert f"workspace={workspace.resolve()}" in completed.stdout
    assert "ros=1 install=1 env=loaded" in completed.stdout
    assert f"child_workspace={workspace.resolve()} child_env=loaded" in completed.stdout


def test_activate_reports_actionable_ros_setup_error(tmp_path: Path):
    workspace, _ = _make_activation_fixture(tmp_path)
    missing_ros = tmp_path / "missing-ros" / "setup.bash"
    script = f"""
unset WORKSPACE
export EMBODIED_ROS_SETUP={shlex.quote(str(missing_ros))}
source {shlex.quote(str(workspace / 'scripts' / 'activate.sh'))}
"""

    completed = _run_bash(script)

    assert completed.returncode != 0
    assert "ROS 2" in completed.stderr
    assert str(missing_ros) in completed.stderr
    assert "bootstrap.sh" in completed.stderr


def test_activate_reports_actionable_missing_install_error(tmp_path: Path):
    workspace, ros_setup = _make_activation_fixture(tmp_path, with_install=False)
    script = f"""
unset WORKSPACE
export EMBODIED_ROS_SETUP={shlex.quote(str(ros_setup))}
source {shlex.quote(str(workspace / 'scripts' / 'activate.sh'))}
"""

    completed = _run_bash(script)

    assert completed.returncode != 0
    assert str(workspace / "install" / "setup.bash") in completed.stderr
    assert "colcon build --symlink-install" in completed.stderr


def test_public_demo_entrypoints_share_workspace_resolver():
    """公共入口不能再次写死主工作区，否则 worktree 会运行错误的 install。"""

    for relative in (
        "scripts/activate.sh",
        "scripts/acceptance_test.sh",
        "scripts/bootstrap.sh",
        "scripts/setup_frontier_exploration.sh",
        "scripts/voice_slam_nav_showcase.sh",
    ):
        text = (ROOT / relative).read_text(encoding="utf-8")
        assert "lifecycle_utils.sh" in text, relative
        assert 'embodied_resolve_workspace "${BASH_SOURCE[0]}"' in text, relative
        assert "WORKSPACE=\"${WORKSPACE:-/home/ubuntu/embodied_agent_ws}\"" not in text


def test_autonomous_slam_entries_run_workspace_doctor():
    acceptance = (ROOT / "scripts" / "acceptance_test.sh").read_text(encoding="utf-8")
    showcase = (ROOT / "scripts" / "voice_slam_nav_showcase.sh").read_text(
        encoding="utf-8"
    )
    helper = WORKSPACE_HELPER.read_text(encoding="utf-8")

    assert "embodied_workspace_doctor()" in helper
    assert "RUN_AUTOMATIC_MISSION" in helper
    assert "ros2 pkg prefix explore_lite" in helper
    assert "slam-nav-showcase-stage)\n    embodied_workspace_doctor true" in acceptance
    assert "slam-autonomous-mission)\n    embodied_workspace_doctor true" in acceptance
    assert "embodied_workspace_doctor true" in showcase
