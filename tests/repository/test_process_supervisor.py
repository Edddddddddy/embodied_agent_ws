"""subprocess 进程组与会话回收的独立契约。"""

from pathlib import Path
import os
import time

import pytest

from tools.acceptance.errors import AcceptanceSessionError
from tools.acceptance.process_supervisor import (
    RunResult,
    SessionCleanupResult,
    SubprocessProcessAdapter,
)


def test_timeout_keeps_graceful_child_returncode(tmp_path: Path):
    adapter = SubprocessProcessAdapter()

    result = adapter.run(
        ["bash", "-c", "trap 'exit 0' TERM; sleep 60"],
        cwd=tmp_path,
        env=os.environ,
        timeout_s=0.1,
    )

    assert result == RunResult(returncode=0, timed_out=True)


def test_adapter_reaps_a_spawned_process_group(tmp_path: Path):
    adapter = SubprocessProcessAdapter()
    adapter.begin_session()
    handle = adapter.spawn(
        ["bash", "-c", "trap 'exit 0' TERM; while true; do sleep 0.1; done"],
        cwd=tmp_path,
        env=os.environ,
        log_path=tmp_path / "worker.log",
    )
    process_id = handle.process.pid
    time.sleep(0.05)

    stopped = adapter.stop(handle, grace_s=0.5)
    cleanup = adapter.finish_session(grace_s=0.5)

    assert stopped.returncode == 0
    assert stopped.graceful is True
    assert cleanup == SessionCleanupResult()
    with pytest.raises(ProcessLookupError):
        os.kill(process_id, 0)


def test_adapter_rejects_nested_session_ownership():
    adapter = SubprocessProcessAdapter()
    adapter.begin_session()
    try:
        with pytest.raises(AcceptanceSessionError, match="already active"):
            adapter.begin_session()
    finally:
        adapter.finish_session(grace_s=0.0)
