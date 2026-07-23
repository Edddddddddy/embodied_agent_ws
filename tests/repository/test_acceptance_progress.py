from __future__ import annotations

import io
import time

from tools.acceptance.progress import AcceptanceProgress


def test_acceptance_progress_keeps_long_gate_observable() -> None:
    """重型门禁等待期间必须持续输出，不能让终端看起来像卡死。"""

    output = io.StringIO()
    progress = AcceptanceProgress(
        label="slam-nav-e2e",
        total_stages=6,
        heartbeat_s=0.01,
        stream=output,
    )

    progress.start(
        session_id="session-1",
        timeout_s=900,
        log_path="/tmp/runtime.log",
        detail_supplier=lambda: "phase=automatic_mapping",
    )
    deadline = time.monotonic() + 0.3
    while "RUNNING" not in output.getvalue() and time.monotonic() < deadline:
        time.sleep(0.005)
    progress.stage(2, "mapping", "frontier exploration running")
    progress.stop("PASS")

    rendered = output.getvalue()
    assert "[slam-nav-e2e] START" in rendered
    assert "timeout=900s" in rendered
    assert "log=/tmp/runtime.log" in rendered
    assert "[slam-nav-e2e] RUNNING" in rendered
    assert "phase=automatic_mapping" in rendered
    assert "[slam-nav-e2e][2/6] mapping" in rendered
    assert "[slam-nav-e2e] PASS" in rendered
