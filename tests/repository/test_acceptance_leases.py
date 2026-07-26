"""ROS domain 与证据目录 lease 的独立契约。"""

from pathlib import Path

import pytest

from tools.acceptance.errors import (
    ArtifactLeaseUnavailable,
    DomainLeaseUnavailable,
)
from tools.acceptance.leases import (
    ArtifactDirectoryLeasePool,
    RosDomainLeasePool,
)


def test_domain_lease_pool_prevents_parallel_collision(tmp_path: Path):
    pool = RosDomainLeasePool(tmp_path / "domains", first=220, last=221)
    first = pool.acquire(session_id="first")
    second = pool.acquire(session_id="second")
    try:
        assert (first.domain_id, second.domain_id) == (220, 221)
        with pytest.raises(DomainLeaseUnavailable):
            pool.acquire(session_id="third")
    finally:
        first.release()
        second.release()

    reused = pool.acquire(session_id="reused", preferred=220)
    reused.release()


def test_artifact_lease_blocks_a_second_writer_and_can_be_reused(tmp_path: Path):
    pool = ArtifactDirectoryLeasePool(tmp_path / "artifact-locks")
    target = tmp_path / "evidence"
    first = pool.acquire(target=target, session_id="first")
    try:
        with pytest.raises(ArtifactLeaseUnavailable, match="already in use"):
            pool.acquire(target=target, session_id="second")
    finally:
        first.release()

    reused = pool.acquire(target=target, session_id="reused")
    reused.release()
