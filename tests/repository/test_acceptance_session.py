"""AcceptanceSession 深 Module 的纯 Python 行为契约。"""

from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import threading
import time

import pytest

from repository_test_support import ROOT
import tools.acceptance.session as session_module
from tools.acceptance.resource_watchdog import (
    ResourceWatchdogConfig,
    ResourceWatchdogSummary,
)
from tools.acceptance.scenarios import unknown_world_slam_e2e
from tools.acceptance.scenarios.slam_nav_e2e import verify_report
from tools.acceptance.scenarios.unknown_world_run_profile import (
    UnknownWorldRunProfile,
)
from tools.acceptance.session import (
    ArtifactLeaseUnavailable,
    AcceptanceCommandError,
    AcceptanceResourceExhaustion,
    AcceptanceSession,
    AcceptanceSessionConfig,
    AcceptanceSessionError,
    DomainLeaseUnavailable,
    RosDomainLeasePool,
    RosEnvironmentIsolation,
    RunResult,
    SessionCleanupResult,
    StopResult,
    SubprocessProcessAdapter,
)
from tools.acceptance.visual_runtime import LinuxMemorySnapshot


class FakeProcessAdapter:
    def __init__(self, run_result: RunResult = RunResult(0)) -> None:
        self.run_result = run_result
        self.spawned: list[dict] = []
        self.runs: list[dict] = []
        self.stopped: list[tuple[object, float]] = []
        self.session_started = False

    def spawn(self, argv, *, cwd, env, log_path):
        handle = object()
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_path.write_text("fake runtime log\n", encoding="utf-8")
        self.spawned.append(
            {
                "argv": tuple(argv),
                "cwd": cwd,
                "env": dict(env),
                "log_path": log_path,
                "handle": handle,
            }
        )
        return handle

    def run(self, argv, *, cwd, env, timeout_s):
        self.runs.append(
            {
                "argv": tuple(argv),
                "cwd": cwd,
                "env": dict(env),
                "timeout_s": timeout_s,
            }
        )
        return self.run_result

    def stop(self, handle, *, grace_s):
        self.stopped.append((handle, grace_s))
        return StopResult(returncode=0, graceful=True, killed=False)

    def begin_session(self):
        self.session_started = True

    def finish_session(self, *, grace_s):
        assert self.session_started
        self.session_started = False
        return SessionCleanupResult()


def _config(tmp_path: Path, **overrides) -> AcceptanceSessionConfig:
    values = {
        "name": "test-session",
        "workspace": tmp_path,
        "artifact_root": tmp_path / "artifacts",
        "timeout_s": 30.0,
        "session_id": "test-session-1",
        "domain_first": 220,
        "domain_last": 221,
        "termination_grace_s": 0.25,
        "lock_root": tmp_path / "locks",
    }
    values.update(overrides)
    return AcceptanceSessionConfig(**values)


def test_domain_lease_pool_prevents_parallel_domain_collision(tmp_path):
    pool = RosDomainLeasePool(tmp_path / "leases", 220, 221)
    first = pool.acquire(session_id="first")
    second = pool.acquire(session_id="second")
    try:
        assert first.domain_id == 220
        assert second.domain_id == 221
        with pytest.raises(DomainLeaseUnavailable):
            pool.acquire(session_id="third")
    finally:
        first.release()
        second.release()

    reused = pool.acquire(session_id="reused", preferred=220)
    try:
        assert reused.domain_id == 220
    finally:
        reused.release()


def test_acceptance_session_owns_environment_process_cleanup_and_manifest(tmp_path):
    adapter = FakeProcessAdapter()
    session = AcceptanceSession(
        _config(tmp_path, environment={"SCENARIO_FLAG": "enabled"}),
        process_adapter=adapter,
        base_environment={},
    )

    with session as active:
        handle = active.spawn("orchestrator", ["ros2", "run", "demo"])
        assert active.run(["python3", "probe.py"], timeout_s=5.0) == 0
        assert active.environment["ROS_DOMAIN_ID"] == "220"
        assert active.environment["SCENARIO_FLAG"] == "enabled"
        assert active.environment["FASTDDS_BUILTIN_TRANSPORTS"] == "UDPv4"
        assert active.environment["GZ_PARTITION"].startswith(
            "embodied_agent_220_test-session-1"
        )

    assert adapter.stopped == [(handle, 0.25)]
    manifest = json.loads(session.manifest_path.read_text(encoding="utf-8"))
    assert manifest["outcome"] == "passed"
    assert manifest["cleanup_complete"] is True
    assert manifest["orphan_cleanup"]["remaining_pids"] == []
    assert manifest["domain_id"] == 220
    assert manifest["processes"][0]["role"] == "orchestrator"
    assert manifest["processes"][0]["stop"]["graceful"] is True
    assert manifest["commands"][0]["returncode"] == 0


def test_acceptance_session_streams_resource_evidence_into_manifest(tmp_path):
    session = AcceptanceSession(
        _config(
            tmp_path,
            resource_watchdog=ResourceWatchdogConfig(interval_s=60.0),
        ),
        process_adapter=FakeProcessAdapter(),
        base_environment={},
    )

    with session:
        pass

    manifest = json.loads(session.manifest_path.read_text(encoding="utf-8"))
    resources = manifest["resources"]
    assert resources["sample_count"] >= 1
    assert resources["failure"] is None
    assert resources["latest"]["mem_available_mib"] > 0
    assert Path(resources["samples_path"]).is_file()


def test_late_watchdog_failure_during_close_does_not_interrupt_cleanup(
    tmp_path, monkeypatch
):
    failure = "resource_exhaustion: late failure during watchdog stop"
    memory = LinuxMemorySnapshot(
        total_kib=8 * 1024 * 1024,
        available_kib=512 * 1024,
        swap_total_kib=2 * 1024 * 1024,
        swap_free_kib=128 * 1024,
    )

    class LateFailureWatchdog:
        """模拟 stop/join 窗口里才到达的最后一次资源采样。"""

        def __init__(
            self,
            *,
            config,
            session_id,
            output_path,
            on_failure,
        ):
            del config, session_id
            self.output_path = output_path
            self._on_failure = on_failure
            self._failure = None

        def start(self):
            self.output_path.parent.mkdir(parents=True, exist_ok=True)
            self.output_path.write_text("", encoding="utf-8")

        def stop(self):
            self._failure = failure
            self._on_failure(failure)

        @property
        def summary(self):
            return ResourceWatchdogSummary(
                sample_count=1,
                latest_memory=memory,
                peak_session_rss_kib=256 * 1024,
                latest_session_process_count=3,
                failure=self._failure,
            )

    kill_calls = []
    monkeypatch.setattr(session_module, "ResourceWatchdog", LateFailureWatchdog)
    monkeypatch.setattr(
        session_module.os,
        "kill",
        lambda process_id, signal_number: kill_calls.append(
            (process_id, signal_number)
        ),
    )
    adapter = FakeProcessAdapter()
    session = AcceptanceSession(
        _config(
            tmp_path,
            resource_watchdog=ResourceWatchdogConfig(interval_s=60.0),
        ),
        process_adapter=adapter,
        base_environment={},
    )

    with pytest.raises(AcceptanceResourceExhaustion, match="late failure"):
        with session as active:
            handle = active.spawn("orchestrator", ["ros2", "run", "demo"])

    # close() 必须先进入不可中断区，再停止 watchdog；迟到 callback 不能发
    # SIGTERM，且进程、subreaper、manifest 三项收尾都要完整执行。
    assert kill_calls == []
    assert adapter.stopped == [(handle, 0.25)]
    assert adapter.session_started is False
    manifest = json.loads(session.manifest_path.read_text(encoding="utf-8"))
    assert manifest["outcome"] == "failed"
    assert manifest["cleanup_complete"] is True
    assert failure in manifest["error"]
    assert manifest["resources"]["failure"] == failure


def test_resource_watchdog_session_is_rejected_outside_main_thread(tmp_path):
    adapter = FakeProcessAdapter()
    session = AcceptanceSession(
        _config(
            tmp_path,
            resource_watchdog=ResourceWatchdogConfig(interval_s=60.0),
        ),
        process_adapter=adapter,
        base_environment={},
    )
    errors = []

    def open_session():
        try:
            with session:
                pass
        except BaseException as error:
            errors.append(error)

    worker = threading.Thread(target=open_session)
    worker.start()
    worker.join(timeout=2.0)

    assert worker.is_alive() is False
    assert len(errors) == 1
    assert isinstance(errors[0], AcceptanceSessionError)
    assert "main thread" in str(errors[0])
    assert adapter.session_started is False
    assert session.manifest_path.exists() is False


def test_session_overrides_ambient_transport_partitions(tmp_path):
    adapter = FakeProcessAdapter()
    session = AcceptanceSession(
        _config(tmp_path, inherit_ros_domain_id=False),
        process_adapter=adapter,
        base_environment={
            "ROS_DOMAIN_ID": "30",
            "GZ_PARTITION": "stale-shared-partition",
            "IGN_PARTITION": "stale-shared-partition",
            "FASTDDS_BUILTIN_TRANSPORTS": "SHM",
        },
    )

    with session as active:
        expected = "embodied_agent_220_test-session-1"
        assert active.domain_id == 220
        assert active.environment["GZ_PARTITION"] == expected
        assert active.environment["IGN_PARTITION"] == expected
        assert active.environment["FASTDDS_BUILTIN_TRANSPORTS"] == "UDPv4"


def test_session_removes_ambient_priors_and_audits_only_allowlisted_environment(
    tmp_path,
):
    adapter = FakeProcessAdapter()
    session = AcceptanceSession(
        _config(
            tmp_path,
            environment={"NAV2_INITIAL_X": "0.0", "PROFILE": "unknown_world"},
            unset_environment_keys=(
                "EMBODIED_NAV2_PLACES_FILE",
                "NAV2_MAP",
            ),
            manifest_environment_keys=("NAV2_INITIAL_X", "PROFILE"),
        ),
        process_adapter=adapter,
        base_environment={
            "EMBODIED_NAV2_PLACES_FILE": "/tmp/stale-places.yaml",
            "NAV2_MAP": "/tmp/stale-map.yaml",
            "NAV2_INITIAL_X": "9.9",
            "SECRET_API_KEY": "must-not-enter-manifest",
        },
    )

    with session as active:
        assert "EMBODIED_NAV2_PLACES_FILE" not in active.environment
        assert "NAV2_MAP" not in active.environment
        assert active.environment["NAV2_INITIAL_X"] == "0.0"

    manifest = json.loads(session.manifest_path.read_text(encoding="utf-8"))
    assert manifest["environment"] == {
        "EMBODIED_NAV2_PLACES_FILE": None,
        "NAV2_INITIAL_X": "0.0",
        "NAV2_MAP": None,
        "PROFILE": "unknown_world",
    }
    assert "SECRET_API_KEY" not in session.manifest_path.read_text(
        encoding="utf-8"
    )


def test_isolated_ros_environment_rejects_ambient_nav2_but_keeps_frontier(
    tmp_path,
):
    def install_package(prefix: Path, package: str) -> Path:
        marker = (
            prefix
            / "share/ament_index/resource_index/packages"
            / package
        )
        marker.parent.mkdir(parents=True, exist_ok=True)
        marker.touch()
        return prefix

    workspace_install = tmp_path / "feature" / "install"
    project_prefix = install_package(
        workspace_install / "embodied_slam_tools", "embodied_slam_tools"
    )
    system_prefix = tmp_path / "opt/ros/jazzy"
    for package in ("nav2_bringup", "nav2_lifecycle_manager", "nav2_util"):
        install_package(system_prefix, package)
    external_install = tmp_path / "nav2_ws/install"
    external_nav2 = [
        install_package(external_install / package, package)
        for package in ("nav2_bringup", "nav2_lifecycle_manager", "nav2_util")
    ]
    # 模拟同一外部 colcon 工作区里既有不可信 Nav2，也有白名单 Frontier 的
    # isolated install。剔除 Nav2 时仍必须保留 Frontier 自己的 build 路径。
    frontier_install = external_install
    frontier_prefix = install_package(
        frontier_install / "explore_lite", "explore_lite"
    )
    frontier_build = external_install.parent / "build/explore_lite"
    adapter = FakeProcessAdapter()
    session = AcceptanceSession(
        _config(
            tmp_path,
            workspace=workspace_install.parent,
            ros_environment_isolation=RosEnvironmentIsolation(
                system_prefix=system_prefix
            ),
        ),
        process_adapter=adapter,
        base_environment={
            "AMENT_PREFIX_PATH": os.pathsep.join(
                [
                    str(project_prefix),
                    *(str(prefix) for prefix in external_nav2),
                    str(frontier_prefix),
                    str(system_prefix),
                ]
            ),
            "CMAKE_PREFIX_PATH": os.pathsep.join(
                [
                    str(project_prefix),
                    *(str(prefix) for prefix in external_nav2),
                    str(frontier_prefix),
                    str(system_prefix),
                ]
            ),
            "COLCON_PREFIX_PATH": os.pathsep.join(
                [
                    str(workspace_install),
                    str(external_install),
                ]
            ),
            "LD_LIBRARY_PATH": os.pathsep.join(
                [
                    str(external_nav2[-1] / "lib"),
                    str(frontier_prefix / "lib"),
                    str(system_prefix / "lib"),
                ]
            ),
            "PATH": os.pathsep.join(
                [
                    str(external_nav2[0] / "bin"),
                    str(frontier_prefix / "bin"),
                    "/usr/bin",
                ]
            ),
            "PYTHONPATH": os.pathsep.join(
                [
                    str(external_install.parent / "build/nav2_simple_commander"),
                    str(frontier_build),
                    str(project_prefix / "lib/python3.12/site-packages"),
                    str(system_prefix / "lib/python3.12/site-packages"),
                ]
            ),
        },
    )

    with session as active:
        for prefix in external_nav2:
            assert str(prefix) not in active.environment["AMENT_PREFIX_PATH"]
            assert str(prefix) not in active.environment["CMAKE_PREFIX_PATH"]
        assert str(external_install) not in active.environment[
            "COLCON_PREFIX_PATH"
        ]
        assert str(external_nav2[-1] / "lib") not in active.environment[
            "LD_LIBRARY_PATH"
        ]
        assert str(external_nav2[0] / "bin") not in active.environment["PATH"]
        assert "build/nav2_simple_commander" not in active.environment[
            "PYTHONPATH"
        ]
        assert str(project_prefix) in active.environment["AMENT_PREFIX_PATH"]
        assert str(system_prefix) in active.environment["AMENT_PREFIX_PATH"]
        assert str(frontier_prefix) in active.environment["AMENT_PREFIX_PATH"]
        assert str(frontier_prefix / "lib") in active.environment[
            "LD_LIBRARY_PATH"
        ]
        assert str(frontier_build) in active.environment["PYTHONPATH"]

    manifest = json.loads(session.manifest_path.read_text(encoding="utf-8"))
    provenance = manifest["ros_environment"]
    assert provenance["isolated"] is True
    assert provenance["package_prefixes"] == {
        "explore_lite": str(frontier_prefix),
        "explore_lite_msgs": None,
        "nav2_bringup": str(system_prefix),
        "nav2_lifecycle_manager": str(system_prefix),
        "nav2_util": str(system_prefix),
    }
    retained_paths = [
        value
        for values in provenance["paths"].values()
        for value in values
    ]
    assert all(
        str(prefix) not in value
        for prefix in external_nav2
        for value in retained_paths
    )
    assert all(
        "build/nav2_simple_commander" not in value
        for value in retained_paths
    )
    assert str(frontier_prefix) in retained_paths
    assert str(frontier_build) in retained_paths


def test_isolated_ros_environment_fails_for_unsafe_merged_frontier_prefix(
    tmp_path,
):
    def install_package(prefix: Path, package: str) -> None:
        marker = (
            prefix
            / "share/ament_index/resource_index/packages"
            / package
        )
        marker.parent.mkdir(parents=True, exist_ok=True)
        marker.touch()

    system_prefix = tmp_path / "opt/ros/jazzy"
    for package in ("nav2_bringup", "nav2_lifecycle_manager", "nav2_util"):
        install_package(system_prefix, package)
    merged_prefix = tmp_path / "mixed_ws/install"
    install_package(merged_prefix, "explore_lite")
    install_package(merged_prefix, "nav2_bringup")
    session = AcceptanceSession(
        _config(
            tmp_path,
            ros_environment_isolation=RosEnvironmentIsolation(
                system_prefix=system_prefix
            ),
        ),
        process_adapter=FakeProcessAdapter(),
        base_environment={
            "AMENT_PREFIX_PATH": os.pathsep.join(
                (str(merged_prefix), str(system_prefix))
            ),
        },
    )

    with pytest.raises(
        AcceptanceSessionError,
        match="external frontier prefix also provides non-allowlisted packages",
    ):
        session.__enter__()


@pytest.mark.parametrize(
    "profile",
    (
        UnknownWorldRunProfile.synthetic(),
        UnknownWorldRunProfile.live_voice("offline"),
    ),
)
def test_public_unknown_world_profiles_enable_ros_environment_isolation(
    monkeypatch,
    profile,
):
    observed = []

    class ConfigCaptured(Exception):
        pass

    def capture_config(config):
        observed.append(config)
        raise ConfigCaptured

    monkeypatch.setattr(
        unknown_world_slam_e2e,
        "AcceptanceSession",
        capture_config,
    )

    with pytest.raises(ConfigCaptured):
        unknown_world_slam_e2e.run(profile)

    assert observed[0].ros_environment_isolation == RosEnvironmentIsolation()


def test_artifact_directory_lease_blocks_concurrent_writers(tmp_path):
    artifact_dir = tmp_path / "shared-evidence"
    first = AcceptanceSession(
        _config(tmp_path, artifact_dir=artifact_dir, session_id="first"),
        process_adapter=FakeProcessAdapter(),
        base_environment={},
    )
    second = AcceptanceSession(
        _config(tmp_path, artifact_dir=artifact_dir, session_id="second"),
        process_adapter=FakeProcessAdapter(),
        base_environment={},
    )

    with first:
        with pytest.raises(ArtifactLeaseUnavailable, match="already in use"):
            second.__enter__()

    with second:
        pass


@pytest.mark.parametrize(
    ("result", "message"),
    [
        (RunResult(7), "command failed with exit 7"),
        (RunResult(-9, timed_out=True), "command timed out"),
    ],
)
def test_failed_command_cleans_children_and_preserves_failure_evidence(
    tmp_path, capsys, result, message
):
    adapter = FakeProcessAdapter(result)
    session = AcceptanceSession(
        _config(tmp_path), process_adapter=adapter, base_environment={}
    )

    with pytest.raises(AcceptanceCommandError, match=message):
        with session as active:
            active.spawn("orchestrator", ["ros2", "run", "demo"])
            active.run(["python3", "probe.py"], timeout_s=5.0)

    manifest = json.loads(session.manifest_path.read_text(encoding="utf-8"))
    assert manifest["outcome"] == "failed"
    assert manifest["cleanup_complete"] is True
    assert manifest["error"]
    assert manifest["commands"][0]["error"]
    assert len(adapter.stopped) == 1
    assert "fake runtime log" in capsys.readouterr().err


def test_unreaped_process_makes_cleanup_and_session_fail(tmp_path):
    class UnreapedProcessAdapter(FakeProcessAdapter):
        def stop(self, handle, *, grace_s):
            self.stopped.append((handle, grace_s))
            return StopResult(returncode=None, graceful=False, killed=True)

    session = AcceptanceSession(
        _config(tmp_path),
        process_adapter=UnreapedProcessAdapter(),
        base_environment={},
    )

    with pytest.raises(AcceptanceSessionError, match="cleanup failed"):
        with session as active:
            active.spawn("unreaped", ["demo"])

    manifest = json.loads(session.manifest_path.read_text(encoding="utf-8"))
    assert manifest["outcome"] == "failed"
    assert manifest["cleanup_complete"] is False
    assert "could not be reaped" in manifest["error"]


def test_subprocess_adapter_reaps_a_real_process_group(tmp_path):
    session = AcceptanceSession(
        _config(tmp_path, termination_grace_s=0.5),
        base_environment=os.environ,
    )

    with session as active:
        handle = active.spawn(
            "worker",
            ["bash", "-c", "trap 'exit 0' TERM; while true; do sleep 0.1; done"],
        )
        process_id = handle.process.pid
        time.sleep(0.05)
        os.kill(process_id, 0)

    with pytest.raises(ProcessLookupError):
        os.kill(process_id, 0)
    manifest = json.loads(session.manifest_path.read_text(encoding="utf-8"))
    assert manifest["cleanup_complete"] is True
    assert manifest["processes"][0]["stop"]["graceful"] is True


def test_subprocess_timeout_preserves_a_graceful_zero_returncode(tmp_path):
    adapter = SubprocessProcessAdapter()

    result = adapter.run(
        ["bash", "-c", "trap 'exit 0' TERM; sleep 60"],
        cwd=tmp_path,
        env=os.environ,
        timeout_s=0.1,
    )

    assert result == RunResult(returncode=0, timed_out=True)


def test_subprocess_adapter_reaps_descendant_that_created_a_new_session(tmp_path):
    nested_pid_file = tmp_path / "nested.pid"
    child_script = "import time; time.sleep(60)"
    parent_script = f"""
import pathlib, subprocess, sys, threading, time

def launch_from_worker_thread():
    child = subprocess.Popen(
        [sys.executable, "-c", {child_script!r}], start_new_session=True
    )
    pathlib.Path({str(nested_pid_file)!r}).write_text(str(child.pid))
    time.sleep(60)

threading.Thread(target=launch_from_worker_thread).start()
time.sleep(60)
"""
    session = AcceptanceSession(
        _config(tmp_path, termination_grace_s=0.5),
        base_environment=os.environ,
    )

    with session as active:
        active.spawn("parent", [sys.executable, "-c", parent_script])
        deadline = time.monotonic() + 2.0
        nested_pid_text = ""
        while time.monotonic() < deadline:
            if nested_pid_file.is_file():
                # write_text 会先创建目录项再写内容；只等 is_file() 存在一个
                # 很短的 TOCTOU 窗口，负载较高时会读到空串并造成随机失败。
                nested_pid_text = nested_pid_file.read_text().strip()
                if nested_pid_text.isdigit():
                    break
            time.sleep(0.02)
        assert nested_pid_text.isdigit(), "nested child did not publish its pid"
        nested_pid = int(nested_pid_text)
        os.kill(nested_pid, 0)

    deadline = time.monotonic() + 2.0
    while Path(f"/proc/{nested_pid}").exists() and time.monotonic() < deadline:
        state = Path(f"/proc/{nested_pid}/stat").read_text().split()[2]
        if state == "Z":
            break
        time.sleep(0.02)
    stat_path = Path(f"/proc/{nested_pid}/stat")
    if stat_path.exists():
        assert stat_path.read_text().split()[2] == "Z"


def test_subreaper_reaps_setsid_child_after_tracked_parent_already_exited(tmp_path):
    nested_pid_file = tmp_path / "orphan.pid"
    child_ready_file = tmp_path / "orphan.ready"
    child_script = "\n".join(
        [
            "import pathlib, signal, time",
            "signal.signal(signal.SIGTERM, signal.SIG_IGN)",
            f"pathlib.Path({str(child_ready_file)!r}).write_text('ready')",
            "time.sleep(60)",
        ]
    )
    parent_script = "\n".join(
        [
            "import pathlib, subprocess, sys",
            (
                "child = subprocess.Popen([sys.executable, '-c', "
                f"{child_script!r}], start_new_session=True)"
            ),
            f"pathlib.Path({str(nested_pid_file)!r}).write_text(str(child.pid))",
        ]
    )
    session = AcceptanceSession(
        _config(tmp_path, termination_grace_s=0.5),
        base_environment=os.environ,
    )

    with session as active:
        handle = active.spawn("short-parent", [sys.executable, "-c", parent_script])
        deadline = time.monotonic() + 2.0
        while not nested_pid_file.is_file() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert nested_pid_file.is_file(), "orphan child did not publish its pid"
        nested_pid = int(nested_pid_file.read_text().strip())
        assert handle.process.wait(timeout=2.0) == 0
        deadline = time.monotonic() + 2.0
        while not child_ready_file.is_file() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert child_ready_file.is_file(), "orphan child did not become ready"
        os.kill(nested_pid, 0)

    assert not Path(f"/proc/{nested_pid}").exists()
    manifest = json.loads(session.manifest_path.read_text(encoding="utf-8"))
    assert nested_pid in manifest["orphan_cleanup"]["observed_pids"]
    assert manifest["orphan_cleanup"]["killed"] is True
    assert manifest["orphan_cleanup"]["remaining_pids"] == []
    assert manifest["cleanup_complete"] is True


def _valid_report() -> dict:
    return {
        "passed": True,
        "schema_version": 3,
        "session_id": "fresh-session",
        "automatic_mission": True,
        "map_saved": True,
        "final_phase": 12,
        "map": {"known_cells": 7000, "occupied_cells": 200},
        "mapping_path_m": 12.5,
        "frontier_goal_count": 2,
        "exploration_completion_reason": "coverage_plateau",
        "map_provenance": {"yaml_mtime_ns": 101, "image_mtime_ns": 102},
        "dynamic_navigation": {"passed": True, "unique_plan_count": 4},
        "checks": {"fresh_map": True, "final_stop": True},
        "action_results": [
            {"message": "nav2:follow_waypoints:succeeded missed_waypoints=0"}
        ],
        "final_cmd_vel": {"linear_x": 0.0, "angular_z": 0.0},
    }


def test_slam_nav_report_verifier_accepts_fresh_complete_evidence():
    summary = verify_report(
        _valid_report(), expected_session_id="fresh-session", session_start_ns=100
    )

    assert summary == {
        "known_cells": 7000,
        "occupied_cells": 200,
        "mapping_path_m": 12.5,
        "frontier_goal_count": 2,
        "unique_dynamic_plans": 4,
    }


def test_slam_nav_report_verifier_rejects_stale_map():
    report = _valid_report()
    report["map_provenance"]["yaml_mtime_ns"] = 99

    with pytest.raises(ValueError, match="map yaml is not fresh"):
        verify_report(
            report, expected_session_id="fresh-session", session_start_ns=100
        )


def test_slam_nav_e2e_handler_uses_session_scenario_not_legacy_shell():
    handler = (ROOT / "tools/acceptance/handlers/slam_nav.sh").read_text(
        encoding="utf-8"
    )
    scenario = ROOT / "tools/acceptance/scenarios/slam_nav_e2e.py"

    # -u 是现场可观测性契约：session/domain/证据路径必须在重型探针前立即刷新。
    assert "python3 -u -m tools.acceptance.scenarios.slam_nav_e2e" in handler
    assert scenario.is_file()
    assert "AcceptanceSession(" in scenario.read_text(encoding="utf-8")
    assert not (
        ROOT / "scripts/smoke_test_voice_slam_automatic_mission_gazebo.sh"
    ).exists()
