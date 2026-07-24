"""管理 SLAM/Nav2 阶段子进程的启动、存图与可靠回收。"""

from __future__ import annotations

import os
import signal
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class _OwnedProcessTree:
    """manager 启动进程时记录的 Linux 身份，供退出后的安全回收使用。"""

    root_pid: int
    root_start_ticks: str
    process_group_id: int | None
    descendants: dict[int, str] = field(default_factory=dict)
    stop_watcher: threading.Event = field(default_factory=threading.Event)
    watcher: threading.Thread | None = None


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
        persistent_runtime_enabled: bool = False,
    ) -> None:
        self.workspace = workspace
        self.mode = mode
        self.map_prefix = map_prefix
        self.stop_timeout_s = stop_timeout_s
        self.dry_run = dry_run
        self.mission_profile = mission_profile
        self.persistent_runtime_enabled = persistent_runtime_enabled
        self.base_process: subprocess.Popen | None = None
        self.stage_process: subprocess.Popen | None = None
        self.explorer_process: subprocess.Popen | None = None
        self.map_save_process: subprocess.Popen | None = None
        self._base_started_once = False
        self._runtime_params_prepared = False
        self._dry_run_base_live = False
        self._dry_run_stage_started = False
        # 所有 spawn/stop 都经过同一把可重入锁。否则 close() 与 worker
        # 切换 stage 并发时，shutdown 之后仍可能迟到地拉起 Nav2 孤儿进程。
        self._lifecycle_lock = threading.RLock()
        # wrapper 可能先退出，而其 setsid 子进程随后被 WSL init 接管。这里从
        # 启动时持续记录 pid+starttime；回收时绝不只凭一个可复用的 PID 发信号。
        self._ownership_lock = threading.Lock()
        self._owned_process_trees: dict[int, _OwnedProcessTree] = {}
        self._shutdown = False
        self.stage = ""

    def _require_open(self) -> None:
        if self._shutdown:
            raise RuntimeError("stage process manager is shut down")

    @property
    def process(self) -> subprocess.Popen | None:
        """旧编排器的临时兼容别名；新代码应使用 ``stage_process``。"""

        return self.stage_process

    @process.setter
    def process(self, value: subprocess.Popen | None) -> None:
        self.stage_process = value

    def _environment(self) -> dict[str, str]:
        environment = os.environ.copy()
        # 持久模式是本 manager 子树的显式部署契约，不能让用户 shell 中遗留的
        # 同名变量把旧 strict/手工流程悄悄切到另一套 launch。
        environment.pop("SHOWCASE_PERSISTENT_SESSION", None)
        if self.persistent_runtime_enabled:
            environment["SHOWCASE_PERSISTENT_SESSION"] = "true"
        environment.pop("SHOWCASE_SESSION_PARAMS_PREPARED", None)
        if self._runtime_params_prepared:
            environment["SHOWCASE_SESSION_PARAMS_PREPARED"] = "true"
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

    def _base_is_live(self) -> bool:
        if self.dry_run:
            return self._dry_run_base_live
        return self.base_process is not None and self.base_process.poll() is None

    def start_base(self) -> None:
        """启动整场演示唯一的 Gazebo/机器人 base。"""

        with self._lifecycle_lock:
            self._require_open()
            if self._base_started_once:
                raise RuntimeError("base process already running")
            if self.persistent_runtime_enabled:
                self._prepare_runtime_parameters()
            command = self.command("base")
            if self.dry_run:
                print("DRY RUN base:", " ".join(command), flush=True)
                self._base_started_once = True
                self._dry_run_base_live = True
                return
            self.base_process = subprocess.Popen(
                command,
                cwd=self.workspace,
                env=self._environment(),
                start_new_session=True,
            )
            self._register_process_tree(self.base_process)
            self._base_started_once = True

    def _prepare_runtime_parameters(self) -> None:
        """在并发启动 base/stage 前同步生成唯一的一份会话参数."""

        if self._runtime_params_prepared:
            return
        command = self.command("prepare")
        if self.dry_run:
            print("DRY RUN prepare:", " ".join(command), flush=True)
            self._runtime_params_prepared = True
            return
        completed = subprocess.run(
            command,
            cwd=self.workspace,
            env=self._environment(),
            text=True,
            capture_output=True,
            timeout=60.0,
            check=False,
        )
        if completed.stdout:
            print(completed.stdout, end="", flush=True)
        if completed.returncode != 0:
            error = completed.stderr.strip()
            if not error:
                error = (
                    "persistent runtime parameter preparation exited "
                    f"with code {completed.returncode}"
                )
            raise RuntimeError(error)
        self._runtime_params_prepared = True

    def start_mapping(self) -> None:
        """在 live base 上启动仅属于建图阶段的 ROS 节点。"""

        with self._lifecycle_lock:
            self._require_open()
            if not self._base_is_live():
                raise RuntimeError("base process is not running")
            self.start("mapping")

    def prepare_navigation(self, map_path: Path) -> None:
        """在 stage 已静默时绑定保存地图，但不启动新进程。"""

        with self._lifecycle_lock:
            self._require_open()
            if not self._base_is_live():
                raise RuntimeError("base process is not running")
            stage_is_live = (
                self._dry_run_stage_started
                if self.dry_run
                else (
                    self.stage_process is not None
                    and self.stage_process.poll() is None
                )
            )
            if stage_is_live:
                raise RuntimeError("stage process is still running")
            yaml_path = Path(map_path)
            if not yaml_path.is_file():
                raise RuntimeError(f"saved map does not exist: {yaml_path}")
            self.map_prefix = yaml_path.with_suffix("")

    def start(self, stage: str) -> None:
        with self._lifecycle_lock:
            self._require_open()
            # persistent stage 与 base 共用 Gazebo/robot owner；base 已退出时
            # 禁止再启动 mapping/navigation，以免生成没有世界所有者的孤儿图。
            if self.persistent_runtime_enabled and not self._base_is_live():
                raise RuntimeError("base process is not running")
            if self.process is not None and self.process.poll() is None:
                raise RuntimeError(f"stage process already running: {self.stage}")
            if self.process is not None:
                # 已退出 wrapper 的 ownership 仍可能包含 live setsid 后代；必须
                # 在覆盖句柄前消费快照，否则后续 shutdown 再也无法定位它们。
                self._stop_process_tree(
                    self.process,
                    self.stop_timeout_s,
                )
                self.process = None
                self.stage = ""
            command = self.command(stage)
            if self.dry_run:
                if self._dry_run_stage_started:
                    raise RuntimeError(
                        f"stage process already running: {self.stage}"
                    )
                print(f"DRY RUN {stage}:", " ".join(command), flush=True)
                self.stage = stage
                self._dry_run_stage_started = True
                self.process = None
                return
            self.process = subprocess.Popen(
                command,
                cwd=self.workspace,
                env=self._environment(),
                start_new_session=True,
            )
            self._register_process_tree(self.process)
            self.stage = stage

    def save_map(self) -> str:
        with self._lifecycle_lock:
            self._require_open()
            command = self.command("save")
            if self.dry_run:
                print("DRY RUN save:", " ".join(command), flush=True)
                return str(self.map_prefix.with_suffix(".yaml"))
            if (
                self.map_save_process is not None
                and self.map_save_process.poll() is None
            ):
                raise RuntimeError("map save is already running")
            process = subprocess.Popen(
                command,
                cwd=self.workspace,
                env=self._environment(),
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=True,
            )
            self.map_save_process = process
            self._register_process_tree(process)
        try:
            stdout, stderr = process.communicate(timeout=35.0)
        except subprocess.TimeoutExpired as error:
            self._stop_process_tree(process, self.stop_timeout_s)
            raise RuntimeError("map_saver timed out after 35 seconds") from error
        finally:
            # 正常退出的 map_saver 也可能留下脱组 helper；消费启动期快照后再
            # 清空句柄，避免 shutdown 时已经失去所有权线索。
            self._stop_process_tree(process, self.stop_timeout_s)
            with self._lifecycle_lock:
                if self.map_save_process is process:
                    self.map_save_process = None
        if stdout:
            print(stdout, end="", flush=True)
        if process.returncode != 0:
            error = stderr.strip()
            if not error:
                error = f"map_saver exited with code {process.returncode}"
            raise RuntimeError(error)
        yaml_path = self.map_prefix.with_suffix(".yaml")
        image_path = self.map_prefix.with_suffix(".pgm")
        if not yaml_path.is_file() or not image_path.is_file():
            raise RuntimeError("map_saver returned success without YAML/PGM artifacts")
        return str(yaml_path)

    def start_explorer(self, config_path: Path) -> None:
        with self._lifecycle_lock:
            self._require_open()
            if self.persistent_runtime_enabled and not self._base_is_live():
                raise RuntimeError("base process is not running")
            self._start_explorer_unlocked(config_path)

    def _start_explorer_unlocked(self, config_path: Path) -> None:
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
        if self.explorer_process is not None:
            self._stop_process_tree(self.explorer_process, self.stop_timeout_s)
            self.explorer_process = None
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
        self._register_process_tree(self.explorer_process)

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

    def _register_process_tree(self, process: subprocess.Popen) -> None:
        """从 spawn 时观察子树，使 wrapper 退出后仍保留可验证的所有权。"""

        root_start_ticks = self._process_start_ticks(process.pid)
        if root_start_ticks is None:
            # 极短命令可能在 Popen 返回后已经结束；此时不能只凭 PID 建立所有权。
            return
        try:
            process_group_id = os.getpgid(process.pid)
        except (ProcessLookupError, PermissionError):
            process_group_id = None
        # 所有受管进程均以 start_new_session=True 启动。若该约束不成立，
        # 禁止后续使用 killpg，以免误伤调用者所在的共享进程组。
        if process_group_id != process.pid:
            process_group_id = None
        ownership = _OwnedProcessTree(
            root_pid=process.pid,
            root_start_ticks=root_start_ticks,
            process_group_id=process_group_id,
            descendants=self._descendant_identities(process.pid),
        )
        with self._ownership_lock:
            self._owned_process_trees[process.pid] = ownership

        def observe() -> None:
            # wrapper 运行期间持续合并而不是覆盖快照：setsid 子进程一旦被 init
            # 接管，就无法再从 /proc/<wrapper>/children 反向发现。
            while not ownership.stop_watcher.is_set():
                if (
                    self._process_start_ticks(process.pid)
                    != ownership.root_start_ticks
                ):
                    break
                descendants = self._descendant_identities(process.pid)
                if descendants:
                    with self._ownership_lock:
                        if (
                            self._owned_process_trees.get(process.pid)
                            is not ownership
                        ):
                            break
                        ownership.descendants.update(descendants)
                if process.poll() is not None:
                    break
                ownership.stop_watcher.wait(0.01)

        watcher = threading.Thread(
            target=observe,
            name=f"stage-process-tree-{process.pid}",
            daemon=True,
        )
        ownership.watcher = watcher
        watcher.start()

    def _release_process_tree(
        self, process: subprocess.Popen
    ) -> _OwnedProcessTree | None:
        """停止观察并取走所有权快照；调用方随后负责信号与 wait。"""

        with self._ownership_lock:
            ownership = self._owned_process_trees.get(process.pid)
        if ownership is None:
            # 兼容测试或旧调用方直接注入的 Popen：仅当当前 starttime 可读时，
            # 才现场建立一次安全快照，绝不信任孤立的数字 PID。
            root_start_ticks = self._process_start_ticks(process.pid)
            if root_start_ticks is None:
                return None
            try:
                process_group_id = os.getpgid(process.pid)
            except (ProcessLookupError, PermissionError):
                process_group_id = None
            if process_group_id != process.pid:
                process_group_id = None
            return _OwnedProcessTree(
                root_pid=process.pid,
                root_start_ticks=root_start_ticks,
                process_group_id=process_group_id,
                descendants=self._descendant_identities(process.pid),
            )

        if (
            self._process_start_ticks(process.pid)
            == ownership.root_start_ticks
        ):
            ownership.descendants.update(
                self._descendant_identities(process.pid)
            )
        ownership.stop_watcher.set()
        watcher = ownership.watcher
        if watcher is not None and watcher is not threading.current_thread():
            watcher.join(timeout=0.2)
        with self._ownership_lock:
            if self._owned_process_trees.get(process.pid) is ownership:
                del self._owned_process_trees[process.pid]
        return ownership

    @classmethod
    def _signal_owned_process(
        cls, ownership: _OwnedProcessTree, signal_number: int
    ) -> bool:
        """仅在 PID 的 starttime 未变化时向根进程发信号。"""

        if (
            cls._process_start_ticks(ownership.root_pid)
            != ownership.root_start_ticks
        ):
            return False
        try:
            os.kill(ownership.root_pid, signal_number)
        except (ProcessLookupError, PermissionError):
            return False
        return True

    @classmethod
    def _signal_owned_group(
        cls, ownership: _OwnedProcessTree, signal_number: int
    ) -> bool:
        """验证 root 身份及独占 PGID 后才升级为进程组信号。"""

        if ownership.process_group_id != ownership.root_pid:
            return False
        if (
            cls._process_start_ticks(ownership.root_pid)
            != ownership.root_start_ticks
        ):
            return False
        try:
            if os.getpgid(ownership.root_pid) != ownership.process_group_id:
                return False
            os.killpg(ownership.process_group_id, signal_number)
        except (ProcessLookupError, PermissionError):
            return False
        return True

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

    def stop_explorer(self) -> None:
        with self._lifecycle_lock:
            process = self.explorer_process
            self.explorer_process = None
            if self.dry_run:
                return
            self._stop_process_tree(process, 5.0)

    def explorer_exited_unexpectedly(self) -> tuple[bool, int | None]:
        with self._lifecycle_lock:
            if self.dry_run or self.explorer_process is None:
                return False, None
            code = self.explorer_process.poll()
            return code is not None, code

    def _stop_process_tree(
        self, process: subprocess.Popen | None, timeout_s: float
    ) -> None:
        if process is None:
            return

        # 不能在 poll()!=None 时直接返回：wrapper 可能已退出，但其 setsid
        # Gazebo/ROS 子进程仍活着。启动期观察器保存了经过 starttime 校验的身份。
        ownership = self._release_process_tree(process)
        if ownership is None:
            return

        # 先只通知父脚本，让上层 trap 按顺序关闭 launch/monitor；超时后才
        # 升级为进程组信号，避免 Gazebo/LLM 收到双重中断。
        try:
            if process.poll() is None and self._signal_owned_process(
                ownership,
                signal.SIGTERM,
            ):
                try:
                    process.wait(timeout=timeout_s)
                except subprocess.TimeoutExpired:
                    self._signal_owned_group(ownership, signal.SIGTERM)
                    try:
                        process.wait(timeout=3.0)
                    except subprocess.TimeoutExpired:
                        self._signal_owned_group(ownership, signal.SIGKILL)
                        try:
                            process.wait(timeout=3.0)
                        except subprocess.TimeoutExpired:
                            # SIGKILL 后仍未回收通常表示僵尸进程由上层 init
                            # 接管；关闭路径不应掩盖会话结果或阻塞 ROS 节点退出。
                            pass
        finally:
            self._terminate_descendants(ownership.descendants)

    def stop_stage(self) -> None:
        """按 explorer→stage 回收阶段资源，但保留常驻 base。"""

        with self._lifecycle_lock:
            self.stop_explorer()
            process = self.process
            self.process = None
            self.stage = ""
            self._dry_run_stage_started = False
            if self.dry_run:
                return
            self._stop_process_tree(process, self.stop_timeout_s)

    def stop(self) -> None:
        """旧编排器兼容入口；等价于只停止当前 stage。"""

        self.stop_stage()

    def shutdown(self) -> None:
        """整场结束时按 explorer→stage→base 的所有权顺序回收。"""

        with self._lifecycle_lock:
            # 先写 terminal 状态，再回收进程；等待这把锁的迟到 start 在取得锁后
            # 会 fail-close，而不是在 shutdown 返回后重新创建 stage。
            self._shutdown = True
            map_save_process = self.map_save_process
            self.map_save_process = None
            if not self.dry_run:
                # map_saver 也属于 manager 子树。若 Ctrl+C 发生在保存阶段，
                # 先终止它，不能让 close() 固定等待 subprocess.run 的 35 秒。
                self._stop_process_tree(
                    map_save_process,
                    self.stop_timeout_s,
                )
            self.stop_stage()
            process = self.base_process
            self.base_process = None
            if self.dry_run:
                self._dry_run_base_live = False
                return
            self._stop_process_tree(process, self.stop_timeout_s)

    def unexpected_exit(self) -> tuple[str | None, int | None]:
        """返回意外退出的所有者角色；有多个故障时优先报告常驻 base。"""

        with self._lifecycle_lock:
            if self.dry_run:
                return None, None
            owned_processes = (
                ("base", self.base_process),
                ("stage", self.stage_process),
                ("explorer", self.explorer_process),
            )
            for role, process in owned_processes:
                if process is None:
                    continue
                code = process.poll()
                if code is not None:
                    return role, code
            return None, None

    def exited_unexpectedly(self) -> tuple[bool, int | None]:
        with self._lifecycle_lock:
            if self.dry_run or self.process is None:
                return False, None
            code = self.process.poll()
            return code is not None, code
