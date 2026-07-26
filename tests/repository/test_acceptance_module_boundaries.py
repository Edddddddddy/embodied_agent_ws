"""验收基础设施的依赖方向与 facade 兼容契约。"""

from repository_test_support import ROOT
from tools.acceptance import errors, leases, process_supervisor, session


def test_session_facade_reexports_canonical_runtime_types():
    expected = {
        "AcceptanceCommandError": errors.AcceptanceCommandError,
        "AcceptanceResourceExhaustion": errors.AcceptanceResourceExhaustion,
        "AcceptanceSessionError": errors.AcceptanceSessionError,
        "AcceptanceSignalError": errors.AcceptanceSignalError,
        "ArtifactLeaseUnavailable": errors.ArtifactLeaseUnavailable,
        "DomainLeaseUnavailable": errors.DomainLeaseUnavailable,
        "ArtifactDirectoryLease": leases.ArtifactDirectoryLease,
        "ArtifactDirectoryLeasePool": leases.ArtifactDirectoryLeasePool,
        "RosDomainLease": leases.RosDomainLease,
        "RosDomainLeasePool": leases.RosDomainLeasePool,
        "ProcessAdapter": process_supervisor.ProcessAdapter,
        "RunResult": process_supervisor.RunResult,
        "SessionCleanupResult": process_supervisor.SessionCleanupResult,
        "StopResult": process_supervisor.StopResult,
        "SubprocessProcessAdapter": process_supervisor.SubprocessProcessAdapter,
    }

    assert set(expected).issubset(session.__all__)
    for name, canonical_type in expected.items():
        assert getattr(session, name) is canonical_type


def test_low_level_modules_do_not_import_the_session_facade():
    for relative_path in (
        "tools/acceptance/errors.py",
        "tools/acceptance/leases.py",
        "tools/acceptance/process_supervisor.py",
    ):
        source = (ROOT / relative_path).read_text(encoding="utf-8")
        assert "tools.acceptance.session" not in source
