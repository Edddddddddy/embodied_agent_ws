"""捕获 Nav2 已死亡但高层任务仍等待的回归测试。"""

from tools.acceptance.runtime_log_health import RuntimeLogHealthMonitor


def test_runtime_log_monitor_detects_lifecycle_critical_failure(tmp_path):
    runtime_log = tmp_path / "runtime.log"
    runtime_log.write_text(
        "[INFO] exploration running\n"
        "[ERROR] CRITICAL FAILURE: SERVER collision_monitor IS DOWN "
        "after not receiving a heartbeat for 4000 ms.\n",
        encoding="utf-8",
    )

    failure = RuntimeLogHealthMonitor(runtime_log).check()

    assert failure is not None
    assert "nav2_runtime_unhealthy" in failure
    assert "collision_monitor" in failure


def test_runtime_log_monitor_detects_critical_failure_without_final_newline(
    tmp_path,
):
    runtime_log = tmp_path / "runtime.log"
    runtime_log.write_text(
        "[ERROR] CRITICAL FAILURE: SERVER bt_navigator IS DOWN "
        "after not receiving a heartbeat for 4000 ms.",
        encoding="utf-8",
    )

    failure = RuntimeLogHealthMonitor(runtime_log).check()

    assert failure is not None
    assert "nav2_runtime_unhealthy" in failure
    assert "bt_navigator" in failure


def test_runtime_log_monitor_ignores_controller_jitter_and_reads_incrementally(
    tmp_path,
):
    runtime_log = tmp_path / "runtime.log"
    runtime_log.write_text(
        "[WARN] Control loop missed its desired rate of 20.0000Hz\n",
        encoding="utf-8",
    )
    monitor = RuntimeLogHealthMonitor(runtime_log)

    assert monitor.check() is None
    with runtime_log.open("a", encoding="utf-8") as stream:
        stream.write(
            "[INFO] Have not received a heartbeat from collision_monitor\n"
        )
    # 单次 heartbeat warning 可能只是瞬态；真正关闭服务器的 CRITICAL 才终止。
    assert monitor.check() is None
