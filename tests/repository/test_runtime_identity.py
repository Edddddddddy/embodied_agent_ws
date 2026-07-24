"""持久 Gazebo 会话的 Linux 进程身份取证测试."""

import os
import subprocess
from pathlib import Path

from tools.acceptance.runtime_identity import (
    ProcessIdentity,
    RuntimeCheckpoint,
)


def test_process_identity_is_stable_for_the_same_live_process() -> None:
    """同一活进程的身份在阶段间保持稳定."""
    first = ProcessIdentity.capture(os.getpid())
    second = ProcessIdentity.capture(os.getpid())

    assert first == second
    assert first.pid == os.getpid()
    assert first.start_ticks > 0
    assert first.boot_id
    assert Path(first.executable).is_absolute()
    assert len(first.argv_sha256) == 64


def test_process_identity_becomes_stale_after_process_exit() -> None:
    """退出后的 PID 不得继续被视为原运行时角色."""
    process = subprocess.Popen(['sleep', '30'])
    identity = ProcessIdentity.capture(process.pid)
    try:
        assert identity.is_current()
    finally:
        process.terminate()
        process.wait(timeout=2.0)

    assert not identity.is_current()


def test_process_identity_evidence_contains_no_raw_command_line() -> None:
    """报告只记录命令行摘要，避免凭据随证据文件泄露."""
    identity = ProcessIdentity.capture(os.getpid())

    evidence = identity.as_evidence()

    assert evidence == {
        'pid': identity.pid,
        'start_ticks': identity.start_ticks,
        'boot_id': identity.boot_id,
        'executable': identity.executable,
        'argv_sha256': identity.argv_sha256,
    }


def test_runtime_checkpoint_binds_roles_to_one_evidence_boundary() -> None:
    """同一 checkpoint 必须绑定具名角色与统一时间边界."""
    checkpoint = RuntimeCheckpoint.capture(
        'mapping_ready',
        {'session_orchestrator': os.getpid()},
        wall_ns=123,
    )

    assert checkpoint.as_evidence() == {
        'label': 'mapping_ready',
        'wall_ns': 123,
        'roles': {
            'session_orchestrator': ProcessIdentity.capture(
                os.getpid()
            ).as_evidence()
        },
    }
