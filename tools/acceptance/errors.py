"""验收基础设施共享的稳定异常类型。"""

from __future__ import annotations

import signal


class AcceptanceSessionError(RuntimeError):
    """验收会话无法安全继续。"""


class DomainLeaseUnavailable(AcceptanceSessionError):
    """候选 ROS domain 均被其他验收会话占用。"""


class ArtifactLeaseUnavailable(AcceptanceSessionError):
    """证据目录正在被另一验收会话写入。"""


class AcceptanceCommandError(AcceptanceSessionError):
    """会话内命令失败或超过截止时间。"""


class AcceptanceResourceExhaustion(AcceptanceSessionError):
    """WSL 资源持续低于安全线；会话必须先停车再释放进程。"""


class AcceptanceSignalError(AcceptanceSessionError):
    """SIGTERM/SIGHUP 被转换成可穿过 context manager 的异常。"""

    def __init__(self, signal_number: int) -> None:
        self.signal_number = signal_number
        super().__init__(
            f"acceptance interrupted by {signal.Signals(signal_number).name}"
        )
