"""StageProcessManager 的 Linux 子进程树回收回归测试。"""

from pathlib import Path
import subprocess
import time

from embodied_slam_tools.stage_process_manager import StageProcessManager


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
