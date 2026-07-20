"""管理 SLAM/Nav2 阶段子进程的启动、存图与可靠回收。"""

from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import time


class StageProcessManager:
    """只负责子进程生命周期；状态策略留在 SessionOrchestratorNode。"""

    def __init__(
        self,
        workspace: Path,
        mode: str,
        map_prefix: Path,
        *,
        stop_timeout_s: float,
        dry_run: bool,
        mission_profile: str = "known_world",
    ) -> None:
        self.workspace = workspace
        self.mode = mode
        self.map_prefix = map_prefix
        self.stop_timeout_s = stop_timeout_s
        self.dry_run = dry_run
        self.mission_profile = mission_profile
        self.process: subprocess.Popen | None = None
        self.explorer_process: subprocess.Popen | None = None
        self.stage = ""

    def _environment(self) -> dict[str, str]:
        environment = os.environ.copy()
        environment["WORKSPACE"] = str(self.workspace)
        environment["SHOWCASE_SESSION_DIR"] = str(self.map_prefix.parent)
        environment["SHOWCASE_MAP_PREFIX"] = str(self.map_prefix)
        environment["SLAM_MISSION_PROFILE"] = self.mission_profile
        return environment

    def command(self, stage: str) -> list[str]:
        return [
            "bash",
            str(self.workspace / "scripts/voice_slam_nav_showcase.sh"),
            stage,
            self.mode,
        ]

    def start(self, stage: str) -> None:
        if self.process is not None and self.process.poll() is None:
            raise RuntimeError(f"stage process already running: {self.stage}")
        command = self.command(stage)
        if self.dry_run:
            print("DRY RUN stage:", " ".join(command), flush=True)
            self.stage = stage
            self.process = None
            return
        self.process = subprocess.Popen(
            command,
            cwd=self.workspace,
            env=self._environment(),
            start_new_session=True,
        )
        self.stage = stage

    def save_map(self) -> str:
        command = self.command("save")
        if self.dry_run:
            print("DRY RUN save:", " ".join(command), flush=True)
            return str(self.map_prefix.with_suffix(".yaml"))
        completed = subprocess.run(
            command,
            cwd=self.workspace,
            env=self._environment(),
            text=True,
            capture_output=True,
            timeout=35.0,
            check=False,
        )
        if completed.stdout:
            print(completed.stdout, end="", flush=True)
        if completed.returncode != 0:
            raise RuntimeError(
                completed.stderr.strip()
                or f"map_saver exited with code {completed.returncode}"
            )
        yaml_path = self.map_prefix.with_suffix(".yaml")
        image_path = self.map_prefix.with_suffix(".pgm")
        if not yaml_path.is_file() or not image_path.is_file():
            raise RuntimeError("map_saver returned success without YAML/PGM artifacts")
        return str(yaml_path)

    def start_explorer(self, config_path: Path) -> None:
        if self.dry_run:
            print(
                f"DRY RUN frontier explorer: config={config_path}",
                flush=True,
            )
            return
        check = subprocess.run(
            ["ros2", "pkg", "prefix", "explore_lite"],
            cwd=self.workspace,
            env=self._environment(),
            capture_output=True,
            text=True,
            check=False,
        )
        if check.returncode != 0:
            raise RuntimeError(
                "Explore Lite is not installed; run "
                "bash scripts/setup_frontier_exploration.sh"
            )
        if self.explorer_process is not None and self.explorer_process.poll() is None:
            raise RuntimeError("frontier explorer is already running")
        self.explorer_process = subprocess.Popen(
            [
                "ros2",
                "run",
                "explore_lite",
                "explore",
                "--ros-args",
                "--params-file",
                str(config_path),
            ],
            cwd=self.workspace,
            env=self._environment(),
            start_new_session=True,
        )

    @staticmethod
    def _process_start_ticks(pid: int) -> str | None:
        """读取 Linux 进程身份；starttime 可避免清理时误伤复用后的同号 PID。"""

        try:
            stat = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            return None
        fields = stat[stat.rfind(")") + 2 :].split()
        if len(fields) <= 19 or fields[0] == "Z":
            return None
        return fields[19]

    @classmethod
    def _descendant_identities(cls, root_pid: int) -> dict[int, str]:
        """在父进程退出、子进程被 init 接管前，快照完整进程树。"""

        descendants: dict[int, str] = {}
        pending = [root_pid]
        visited = {root_pid}
        while pending:
            parent = pending.pop()
            try:
                raw = Path(
                    f"/proc/{parent}/task/{parent}/children"
                ).read_text(encoding="utf-8")
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                continue
            for token in raw.split():
                child = int(token)
                if child in visited:
                    continue
                visited.add(child)
                start_ticks = cls._process_start_ticks(child)
                if start_ticks is None:
                    continue
                descendants[child] = start_ticks
                pending.append(child)
        return descendants

    @classmethod
    def _terminate_descendants(
        cls, identities: dict[int, str], timeout_s: float = 3.0
    ) -> None:
        """回收已脱离父进程组的 Gazebo 子进程，保证下一阶段世界可启动。"""

        def alive() -> list[int]:
            return [
                pid
                for pid, start_ticks in identities.items()
                if cls._process_start_ticks(pid) == start_ticks
            ]

        for pid in reversed(alive()):
            try:
                os.kill(pid, signal.SIGTERM)
            except (ProcessLookupError, PermissionError):
                pass
        deadline = time.monotonic() + timeout_s
        while alive() and time.monotonic() < deadline:
            time.sleep(0.05)
        for pid in reversed(alive()):
            try:
                os.kill(pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass

    @staticmethod
    def _stop_process(process: subprocess.Popen | None, timeout_s: float) -> None:
        if process is None or process.poll() is not None:
            return
        # `ros2 run` 是 Python 包装进程，真正的 C++ explore 是其子进程；只 terminate
        # 包装器会留下孤儿 explore，并在定位阶段继续发送 frontier goal。
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout_s)
            return
        except subprocess.TimeoutExpired:
            pass
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=3.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def stop_explorer(self) -> None:
        process = self.explorer_process
        self.explorer_process = None
        if self.dry_run:
            return
        self._stop_process(process, 5.0)

    def explorer_exited_unexpectedly(self) -> tuple[bool, int | None]:
        if self.dry_run or self.explorer_process is None:
            return False, None
        code = self.explorer_process.poll()
        return code is not None, code

    def stop(self) -> None:
        self.stop_explorer()
        process = self.process
        self.process = None
        self.stage = ""
        if self.dry_run or process is None or process.poll() is not None:
            return

        # gz sim 可能创建新的进程组，并在 ros2 launch 退出后被 WSL init 接管。
        # 先快照后停父脚本，才能在阶段切换前可靠回收这些“脱组”子进程。
        descendants = self._descendant_identities(process.pid)

        # 先只通知父脚本，让 continuous_nav2_voice_control.sh 的 trap 按顺序关闭
        # launch/monitor；超时后才升级为进程组信号，避免 Gazebo/LLM 收到双重中断。
        process.terminate()
        try:
            process.wait(timeout=self.stop_timeout_s)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=3.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    # SIGKILL 后仍未回收通常表示僵尸进程由上层 init 接管；关闭路径
                    # 不应因此掩盖真正的会话结果或阻塞 ROS 节点退出。
                    pass
        finally:
            self._terminate_descendants(descendants)

    def exited_unexpectedly(self) -> tuple[bool, int | None]:
        if self.dry_run or self.process is None:
            return False, None
        code = self.process.poll()
        return code is not None, code
