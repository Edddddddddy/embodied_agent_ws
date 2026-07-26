"""StageProcessManager 的 Linux 子进程树回收回归测试。"""

import os
import signal
import subprocess
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from embodied_slam_tools.stage_process_manager import StageProcessManager


def _write_stage_script(workspace: Path) -> Path:
    script = workspace / "scripts" / "voice_slam_nav_showcase.sh"
    script.parent.mkdir(parents=True)
    script.write_text(
        """#!/usr/bin/env bash
set -euo pipefail
if [[ "$1" == "prepare" ]]; then
  printf 'prepare\n' >> "$WORKSPACE/prepare.log"
  exit 0
fi
if [[ "$1" == "save" && "${BLOCK_MAP_SAVE:-false}" == "true" ]]; then
  printf 'save-started\n' >> "$WORKSPACE/map-save.log"
  trap 'exit 143' TERM INT
  while true; do
    sleep 30 &
    wait "$!"
  done
fi
printf '%s|%s|%s|%s\\n' \
  "$1" "$SHOWCASE_MAP_PREFIX" "$$" \
  "${SHOWCASE_PERSISTENT_SESSION:-false}" >> "$WORKSPACE/process.log"
trap 'printf "stop:%s\\n" "$1" >> "$WORKSPACE/lifecycle.log"; exit 0' TERM INT
while true; do
  sleep 30 &
  wait "$!"
done
""",
        encoding="utf-8",
    )
    return script


def _write_fake_ros2(workspace: Path) -> Path:
    executable = workspace / "bin" / "ros2"
    executable.parent.mkdir(parents=True)
    executable.write_text(
        """#!/usr/bin/env bash
set -euo pipefail
if [[ "$1" == "pkg" && "$2" == "prefix" ]]; then
  exit 0
fi
printf 'explorer|%s|%s\\n' "$SHOWCASE_MAP_PREFIX" "$$" >> "$WORKSPACE/process.log"
trap 'printf "stop:explorer\\n" >> "$WORKSPACE/lifecycle.log"; exit 0' TERM INT
while true; do
  sleep 30 &
  wait "$!"
done
""",
        encoding="utf-8",
    )
    executable.chmod(0o755)
    return executable


def _write_detaching_stage_script(workspace: Path) -> Path:
    """创建会先退出 wrapper、但保留独立 session 子进程的最小复现场景。"""

    script = workspace / "scripts" / "voice_slam_nav_showcase.sh"
    script.parent.mkdir(parents=True)
    script.write_text(
        """#!/usr/bin/env bash
set -euo pipefail
setsid sleep 30 &
child_pid="$!"
printf '%s\n' "$child_pid" > "$WORKSPACE/detached-child.pid"
# 给 manager 的所有权观察器一个确定窗口，然后模拟 wrapper 意外提前退出。
sleep 0.3
exit 0
""",
        encoding="utf-8",
    )
    return script


def _wait_for_log_lines(path: Path, count: int) -> list[str]:
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        if path.is_file():
            lines = path.read_text(encoding="utf-8").splitlines()
            if len(lines) >= count:
                return lines
        time.sleep(0.02)
    raise AssertionError(f"expected {count} process log lines")


def test_base_rejects_duplicate_start_and_mapping_requires_live_base(
    tmp_path: Path, capsys
):
    manager = StageProcessManager(
        tmp_path,
        "offline",
        tmp_path / "session" / "map",
        stop_timeout_s=0.5,
        dry_run=True,
    )

    with pytest.raises(RuntimeError, match="base process is not running"):
        manager.start_mapping()

    manager.start_base()
    with pytest.raises(RuntimeError, match="base process already running"):
        manager.start_base()
    manager.start_mapping()

    output = capsys.readouterr().out
    assert output.count("DRY RUN base:") == 1
    assert output.count("DRY RUN mapping:") == 1
    assert manager.stage == "mapping"

    manager.shutdown()
    with pytest.raises(RuntimeError, match="shut down"):
        manager.start_mapping()
    with pytest.raises(RuntimeError, match="shut down"):
        manager.start_base()


def test_persistent_manager_marks_only_its_child_environment(
    tmp_path: Path,
):
    """新持久路径显式标记子进程，默认旧流程不能被全局污染。"""
    _write_stage_script(tmp_path)
    manager = StageProcessManager(
        tmp_path,
        "offline",
        tmp_path / "session" / "map",
        stop_timeout_s=0.5,
        dry_run=False,
        persistent_runtime_enabled=True,
    )

    try:
        manager.start_base()
        line = _wait_for_log_lines(tmp_path / "process.log", 1)[0]
        assert line.endswith("|true")
        assert "SHOWCASE_PERSISTENT_SESSION" not in os.environ
    finally:
        manager.shutdown()


def test_persistent_base_prepares_session_parameters_once(tmp_path: Path):
    _write_stage_script(tmp_path)
    manager = StageProcessManager(
        tmp_path,
        "offline",
        tmp_path / "session" / "map",
        stop_timeout_s=0.5,
        dry_run=False,
        persistent_runtime_enabled=True,
    )

    try:
        manager.start_base()
        _wait_for_log_lines(tmp_path / "process.log", 1)
        assert (tmp_path / "prepare.log").read_text(
            encoding="utf-8"
        ).splitlines() == ["prepare"]
    finally:
        manager.shutdown()


def test_shutdown_terminates_inflight_map_save(
    tmp_path: Path, monkeypatch
):
    _write_stage_script(tmp_path)
    monkeypatch.setenv("BLOCK_MAP_SAVE", "true")
    manager = StageProcessManager(
        tmp_path,
        "offline",
        tmp_path / "session" / "map",
        stop_timeout_s=0.2,
        dry_run=False,
    )
    errors: list[BaseException] = []

    def save() -> None:
        try:
            manager.save_map()
        except BaseException as error:
            errors.append(error)

    worker = threading.Thread(target=save)
    worker.start()
    _wait_for_log_lines(tmp_path / "map-save.log", 1)

    manager.shutdown()
    worker.join(timeout=2.0)

    assert not worker.is_alive()
    assert errors
    assert manager.map_save_process is None


def test_explicit_navigation_transition_preserves_base_and_replaces_only_stage(
    tmp_path: Path,
):
    _write_stage_script(tmp_path)
    map_yaml = tmp_path / "session" / "fresh_map.yaml"
    map_yaml.parent.mkdir()
    map_yaml.write_text("image: fresh_map.pgm\n", encoding="utf-8")
    manager = StageProcessManager(
        tmp_path,
        "offline",
        tmp_path / "session" / "initial_map",
        stop_timeout_s=0.5,
        dry_run=False,
    )

    try:
        manager.start_base()
        manager.start_mapping()
        _wait_for_log_lines(tmp_path / "process.log", 2)
        base_process = manager.base_process
        mapping_process = manager.stage_process

        # 数据代际屏障属于 orchestrator，manager 故意不提供把 stop/start
        # 藏在一起的快捷方法。
        manager.stop_stage()
        manager.prepare_navigation(map_yaml)
        manager.start("navigation")
        lines = _wait_for_log_lines(tmp_path / "process.log", 3)

        assert base_process is manager.base_process
        assert base_process is not None and base_process.poll() is None
        assert mapping_process is not None and mapping_process.poll() is not None
        assert manager.stage_process is not None
        assert manager.stage_process.pid != mapping_process.pid
        assert manager.stage == "navigation"
        assert lines[-1].startswith(
            f"navigation|{map_yaml.with_suffix('')}|"
        )
    finally:
        manager.shutdown()


def test_prepare_navigation_requires_quiescent_stage_and_fresh_map(
    tmp_path: Path,
    capsys,
):
    map_yaml = tmp_path / "session" / "fresh_map.yaml"
    manager = StageProcessManager(
        tmp_path,
        "offline",
        tmp_path / "session" / "initial_map",
        stop_timeout_s=0.5,
        dry_run=True,
        persistent_runtime_enabled=True,
    )

    with pytest.raises(RuntimeError, match="base process is not running"):
        manager.prepare_navigation(map_yaml)

    manager.start_base()
    manager.start_mapping()
    with pytest.raises(RuntimeError, match="stage process is still running"):
        manager.prepare_navigation(map_yaml)
    manager.stop_stage()
    with pytest.raises(RuntimeError, match="saved map does not exist"):
        manager.prepare_navigation(map_yaml)

    map_yaml.parent.mkdir(parents=True)
    map_yaml.write_text("image: fresh_map.pgm\n", encoding="utf-8")
    manager.prepare_navigation(map_yaml)

    assert manager.map_prefix == map_yaml.with_suffix("")
    assert manager.stage == ""
    assert "DRY RUN navigation:" not in capsys.readouterr().out


def test_shutdown_is_terminal_for_persistent_runtime(tmp_path: Path):
    map_yaml = tmp_path / "session" / "fresh_map.yaml"
    map_yaml.parent.mkdir(parents=True)
    map_yaml.write_text("image: fresh_map.pgm\n", encoding="utf-8")
    manager = StageProcessManager(
        tmp_path,
        "offline",
        tmp_path / "session" / "map",
        stop_timeout_s=0.5,
        dry_run=True,
        persistent_runtime_enabled=True,
    )
    manager.start_base()
    manager.shutdown()

    operations = (
        manager.start_base,
        manager.start_mapping,
        lambda: manager.prepare_navigation(map_yaml),
        lambda: manager.start("navigation"),
        lambda: manager.start_explorer(tmp_path / "explore.yaml"),
        manager.save_map,
    )
    for operation in operations:
        with pytest.raises(RuntimeError, match="shut down"):
            operation()


def test_shutdown_serializes_against_late_stage_start(
    tmp_path: Path, monkeypatch
):
    """shutdown 持锁期间，迟到的 start 必须等待并在终态检查处失败。"""

    map_yaml = tmp_path / "session" / "fresh_map.yaml"
    map_yaml.parent.mkdir(parents=True)
    map_yaml.write_text("image: fresh_map.pgm\n", encoding="utf-8")
    manager = StageProcessManager(
        tmp_path,
        "offline",
        tmp_path / "session" / "map",
        stop_timeout_s=0.5,
        dry_run=False,
        persistent_runtime_enabled=True,
    )
    manager.base_process = SimpleNamespace(poll=lambda: None)
    manager._base_started_once = True
    manager.prepare_navigation(map_yaml)

    shutdown_entered = threading.Event()
    release_shutdown = threading.Event()

    def blocking_stop(_process, _timeout):
        shutdown_entered.set()
        assert release_shutdown.wait(timeout=2.0)

    monkeypatch.setattr(manager, "_stop_process_tree", blocking_stop)
    shutdown_thread = threading.Thread(target=manager.shutdown)
    start_errors = []

    def late_start():
        try:
            manager.start("navigation")
        except Exception as exc:  # noqa: BLE001 - 线程断言需回传异常
            start_errors.append(exc)

    shutdown_thread.start()
    assert shutdown_entered.wait(timeout=2.0)
    start_thread = threading.Thread(target=late_start)
    start_thread.start()
    assert start_thread.is_alive()
    release_shutdown.set()
    shutdown_thread.join(timeout=2.0)
    start_thread.join(timeout=2.0)

    assert not shutdown_thread.is_alive()
    assert not start_thread.is_alive()
    assert len(start_errors) == 1
    assert "shut down" in str(start_errors[0])
    assert manager.stage_process is None


def test_unexpected_exit_identifies_stage_role(tmp_path: Path):
    _write_stage_script(tmp_path)
    manager = StageProcessManager(
        tmp_path,
        "offline",
        tmp_path / "session" / "map",
        stop_timeout_s=0.5,
        dry_run=False,
    )

    try:
        manager.start_base()
        manager.start_mapping()
        _wait_for_log_lines(tmp_path / "process.log", 2)
        stage_process = manager.stage_process
        assert stage_process is not None

        os.killpg(stage_process.pid, signal.SIGTERM)
        stage_process.wait(timeout=2.0)

        assert manager.unexpected_exit() == ("stage", stage_process.returncode)
    finally:
        manager.shutdown()


def test_unexpected_exit_identifies_base_role(tmp_path: Path):
    _write_stage_script(tmp_path)
    manager = StageProcessManager(
        tmp_path,
        "offline",
        tmp_path / "session" / "map",
        stop_timeout_s=0.5,
        dry_run=False,
    )

    try:
        manager.start_base()
        _wait_for_log_lines(tmp_path / "process.log", 1)
        base_process = manager.base_process
        assert base_process is not None

        os.killpg(base_process.pid, signal.SIGTERM)
        base_process.wait(timeout=2.0)

        assert manager.unexpected_exit() == ("base", base_process.returncode)
    finally:
        manager.shutdown()


def test_unexpected_exit_identifies_explorer_role(
    tmp_path: Path, monkeypatch
):
    _write_stage_script(tmp_path)
    fake_ros2 = _write_fake_ros2(tmp_path)
    monkeypatch.setenv("PATH", f"{fake_ros2.parent}:{os.environ['PATH']}")
    manager = StageProcessManager(
        tmp_path,
        "offline",
        tmp_path / "session" / "map",
        stop_timeout_s=0.5,
        dry_run=False,
    )

    try:
        manager.start_base()
        manager.start_mapping()
        manager.start_explorer(tmp_path / "explore.yaml")
        _wait_for_log_lines(tmp_path / "process.log", 3)
        explorer_process = manager.explorer_process
        assert explorer_process is not None

        os.killpg(explorer_process.pid, signal.SIGTERM)
        explorer_process.wait(timeout=2.0)

        assert manager.unexpected_exit() == (
            "explorer",
            explorer_process.returncode,
        )
    finally:
        manager.shutdown()


def test_shutdown_stops_explorer_then_stage_then_base(
    tmp_path: Path, monkeypatch
):
    _write_stage_script(tmp_path)
    fake_ros2 = _write_fake_ros2(tmp_path)
    monkeypatch.setenv("PATH", f"{fake_ros2.parent}:{os.environ['PATH']}")
    manager = StageProcessManager(
        tmp_path,
        "offline",
        tmp_path / "session" / "map",
        stop_timeout_s=0.5,
        dry_run=False,
    )

    try:
        manager.start_base()
        manager.start_mapping()
        manager.start_explorer(tmp_path / "explore.yaml")
        _wait_for_log_lines(tmp_path / "process.log", 3)
        manager.shutdown()

        assert _wait_for_log_lines(tmp_path / "lifecycle.log", 3) == [
            "stop:explorer",
            "stop:mapping",
            "stop:base",
        ]
    finally:
        manager.shutdown()


def test_navigation_start_failure_keeps_live_base(tmp_path: Path):
    _write_stage_script(tmp_path)
    map_yaml = tmp_path / "session" / "fresh_map.yaml"
    map_yaml.parent.mkdir()
    map_yaml.write_text("image: fresh_map.pgm\n", encoding="utf-8")

    class NavigationStartFails(StageProcessManager):
        def command(self, stage: str) -> list[str]:
            if stage == "navigation":
                return [str(tmp_path / "missing-navigation-executable")]
            return super().command(stage)

    manager = NavigationStartFails(
        tmp_path,
        "offline",
        tmp_path / "session" / "map",
        stop_timeout_s=0.5,
        dry_run=False,
    )

    try:
        manager.start_base()
        manager.start_mapping()
        _wait_for_log_lines(tmp_path / "process.log", 2)
        base_process = manager.base_process

        manager.stop_stage()
        manager.prepare_navigation(map_yaml)
        with pytest.raises(FileNotFoundError):
            manager.start("navigation")

        assert manager.base_process is base_process
        assert base_process is not None and base_process.poll() is None
        assert manager.stage_process is None
        assert manager.stage == ""
    finally:
        manager.shutdown()


def test_dry_run_exposes_stage_command_without_starting_process(tmp_path: Path):
    manager = StageProcessManager(
        tmp_path,
        "offline",
        tmp_path / "session" / "map",
        stop_timeout_s=0.5,
        dry_run=True,
    )

    manager.start("mapping")

    assert manager.stage == "mapping"
    assert manager.process is None
    assert manager.command("navigation") == [
        "bash",
        str(tmp_path / "scripts/voice_slam_nav_showcase.sh"),
        "navigation",
        "offline",
    ]
    assert manager.save_map() == str(tmp_path / "session" / "map.yaml")


def test_stop_reaps_child_that_created_a_separate_process_group(tmp_path: Path):
    manager = StageProcessManager(
        tmp_path,
        "offline",
        tmp_path / "map",
        stop_timeout_s=0.5,
        dry_run=False,
    )
    # Gazebo server 会像 setsid sleep 一样脱离父进程组；旧实现只停父组会留下它。
    manager.process = subprocess.Popen(
        ["bash", "-c", "setsid sleep 30 & wait"],
        start_new_session=True,
    )
    try:
        deadline = time.monotonic() + 2.0
        descendants = {}
        while time.monotonic() < deadline:
            descendants = manager._descendant_identities(manager.process.pid)
            if descendants:
                break
            time.sleep(0.02)
        assert descendants

        manager.stop()

        assert all(
            manager._process_start_ticks(pid) != start_ticks
            for pid, start_ticks in descendants.items()
        )
    finally:
        if manager.process is not None and manager.process.poll() is None:
            manager.process.kill()


def test_stop_reaps_detached_child_after_wrapper_already_exited(tmp_path: Path):
    """wrapper 已退出时，仍须按启动时记录的身份安全回收其 setsid 子进程。"""

    _write_detaching_stage_script(tmp_path)
    manager = StageProcessManager(
        tmp_path,
        "offline",
        tmp_path / "map",
        stop_timeout_s=0.2,
        dry_run=False,
    )
    child_pid = 0
    child_start_ticks = None

    try:
        manager.start("mapping")
        pid_file = tmp_path / "detached-child.pid"
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not pid_file.is_file():
            time.sleep(0.01)
        assert pid_file.is_file()

        child_pid = int(pid_file.read_text(encoding="utf-8").strip())
        child_start_ticks = manager._process_start_ticks(child_pid)
        assert child_start_ticks is not None
        # 复现的关键条件：子进程通过 setsid 脱离 wrapper 的进程组。
        assert os.getpgid(child_pid) == child_pid

        wrapper = manager.stage_process
        assert wrapper is not None
        wrapper.wait(timeout=2.0)
        assert wrapper.returncode == 0

        manager.stop_stage()

        deadline = time.monotonic() + 2.0
        while (
            manager._process_start_ticks(child_pid) == child_start_ticks
            and time.monotonic() < deadline
        ):
            time.sleep(0.02)
        assert manager._process_start_ticks(child_pid) != child_start_ticks
        assert manager._owned_process_trees == {}
    finally:
        manager.shutdown()
        # 红测失败时也只按 starttime 精确清理本测试创建的进程，避免误伤复用 PID。
        if (
            child_pid > 0
            and child_start_ticks is not None
            and manager._process_start_ticks(child_pid) == child_start_ticks
        ):
            os.kill(child_pid, signal.SIGKILL)
