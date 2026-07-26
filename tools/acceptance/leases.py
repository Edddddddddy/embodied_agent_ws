"""验收会话的 ROS domain 与证据目录独占 lease。"""

from __future__ import annotations

import fcntl
import hashlib
import json
import os
from dataclasses import dataclass
from pathlib import Path

from tools.acceptance.errors import ArtifactLeaseUnavailable, DomainLeaseUnavailable


@dataclass(slots=True)
class RosDomainLease:
    domain_id: int
    path: Path
    _descriptor: int
    _released: bool = False

    def release(self) -> None:
        if self._released:
            return
        fcntl.flock(self._descriptor, fcntl.LOCK_UN)
        os.close(self._descriptor)
        self._released = True


class RosDomainLeasePool:
    """用 ``flock`` 分配 ROS domain，避免 PID 取模导致并行碰撞。"""

    def __init__(self, root: Path, first: int = 210, last: int = 227) -> None:
        if not (0 <= first <= last <= 232):
            raise ValueError("ROS domain range must be within 0..232")
        self._root = root
        self._first = first
        self._last = last

    def acquire(
        self, *, session_id: str, preferred: int | None = None
    ) -> RosDomainLease:
        if preferred is not None and not 0 <= preferred <= 232:
            raise ValueError("preferred ROS_DOMAIN_ID must be within 0..232")
        self._root.mkdir(parents=True, exist_ok=True)
        candidates = (
            (preferred,)
            if preferred is not None
            else range(self._first, self._last + 1)
        )
        for domain_id in candidates:
            path = self._root / f"ros-domain-{domain_id}.lock"
            descriptor = os.open(path, os.O_RDWR | os.O_CREAT, 0o600)
            try:
                fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                os.close(descriptor)
                continue
            metadata = json.dumps(
                {"pid": os.getpid(), "session_id": session_id},
                ensure_ascii=False,
            ).encode("utf-8")
            os.ftruncate(descriptor, 0)
            os.write(descriptor, metadata)
            os.fsync(descriptor)
            return RosDomainLease(domain_id, path, descriptor)
        requested = (
            str(preferred)
            if preferred is not None
            else f"{self._first}..{self._last}"
        )
        raise DomainLeaseUnavailable(
            f"no free ROS domain lease in {requested}; wait for another acceptance "
            "session or unset ROS_DOMAIN_ID"
        )


@dataclass(slots=True)
class ArtifactDirectoryLease:
    target: Path
    path: Path
    _descriptor: int
    _released: bool = False

    def release(self) -> None:
        if self._released:
            return
        fcntl.flock(self._descriptor, fcntl.LOCK_UN)
        os.close(self._descriptor)
        self._released = True


class ArtifactDirectoryLeasePool:
    """串行化同一证据目录，防止并发会话互删 map/report。"""

    def __init__(self, root: Path) -> None:
        self._root = root

    def acquire(self, *, target: Path, session_id: str) -> ArtifactDirectoryLease:
        resolved = target.resolve()
        digest = hashlib.sha256(os.fsencode(str(resolved))).hexdigest()[:24]
        self._root.mkdir(parents=True, exist_ok=True)
        path = self._root / f"artifact-{digest}.lock"
        descriptor = os.open(path, os.O_RDWR | os.O_CREAT, 0o600)
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            os.close(descriptor)
            raise ArtifactLeaseUnavailable(
                f"acceptance artifact directory is already in use: {resolved}"
            ) from error
        metadata = json.dumps(
            {
                "pid": os.getpid(),
                "session_id": session_id,
                "artifact_dir": str(resolved),
            },
            ensure_ascii=False,
        ).encode("utf-8")
        os.ftruncate(descriptor, 0)
        os.write(descriptor, metadata)
        os.fsync(descriptor)
        return ArtifactDirectoryLease(resolved, path, descriptor)
