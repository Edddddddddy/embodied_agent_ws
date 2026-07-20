from __future__ import annotations

import io
import inspect
import json
import os
import re
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

from tools.acceptance import catalog as acceptance_catalog
from tools.acceptance.catalog import (
    MODE_BY_NAME,
    MODES,
    PUBLIC_MODE_NAMES,
    AcceptanceMode,
    HandlerDomain,
)
from tools.acceptance.cli import main
from tools.acceptance.runner import BashModeRunner


ROOT = Path(__file__).resolve().parents[2]


@dataclass
class RecordingRunner:
    calls: list[tuple[str, list[str]]] = field(default_factory=list)
    result: int = 0

    def run(self, mode: AcceptanceMode, arguments: Sequence[str]) -> int:
        self.calls.append((mode.name, list(arguments)))
        return self.result


def test_public_surface_is_small_stable_and_ordered():
    assert PUBLIC_MODE_NAMES == (
        "core",
        "continuous-offline",
        "continuous-online",
        "gazebo",
        "nav2-stage",
        "slam-nav-e2e",
        "unknown-world-slam-e2e",
        "robotics-gate",
    )
    assert {mode.name for mode in MODES if mode.public} == set(PUBLIC_MODE_NAMES)


def test_acceptance_mode_requires_an_explicit_handler_domain():
    """路由是 catalog 契约，不能再根据 mode 名中的 nav2/slam 等词猜测。"""
    domain_parameter = inspect.signature(AcceptanceMode).parameters["domain"]

    assert domain_parameter.default is inspect.Parameter.empty
    assert not hasattr(acceptance_catalog, "_handler_domain")
    assert all(
        len(spec) == 7 and isinstance(spec[-1], HandlerDomain)
        for spec in acceptance_catalog._MODE_SPECS
    )


def test_every_registered_mode_has_one_handler_in_its_declared_library():
    implementations: dict[str, set[str]] = {}
    for library in {mode.handler_library for mode in MODES}:
        library_path = ROOT / library
        source = library_path.read_text(encoding="utf-8")
        subprocess.run(["bash", "-n", str(library_path)], check=True)
        implementations[library] = set(
            re.findall(r"^(accept_[a-z0-9_]+)\(\) \{", source, re.M)
        )

    assert len(MODE_BY_NAME) == len(MODES)
    for mode in MODES:
        owners = sorted(
            library
            for library, handlers in implementations.items()
            if mode.handler in handlers
        )
        assert owners == [mode.handler_library]
    assert {mode.handler for mode in MODES} == set().union(
        *implementations.values()
    )


def test_legacy_slam_alias_is_not_a_second_e2e_fact_source():
    assert "slam-autonomous-mission" not in MODE_BY_NAME
    assert "slam-autonomous-mission-stage" in MODE_BY_NAME
    assert "slam-nav-e2e" in MODE_BY_NAME


def test_unknown_world_slam_entry_has_distinct_public_evidence_semantics():
    """新旧两个入口必须在帮助中明确区分，避免把已知场景回归当成自主探索证据。"""
    known_world = MODE_BY_NAME["slam-nav-e2e"]
    unknown_world = MODE_BY_NAME["unknown-world-slam-e2e"]

    assert known_world.public is True
    assert known_world.handler == "accept_slam_nav_e2e"
    assert "Known-world deterministic" in known_world.description
    assert unknown_world.public is True
    assert unknown_world.handler == "accept_unknown_world_slam_e2e"
    assert "Unknown-world autonomous exploration" in unknown_world.description
    assert unknown_world.domain is HandlerDomain.SLAM_NAV

    runner = RecordingRunner(result=0)
    status = main(
        ["unknown-world-slam-e2e"],
        runner=runner,
        stdout=io.StringIO(),
        stderr=io.StringIO(),
    )
    assert status == 0
    assert runner.calls == [("unknown-world-slam-e2e", [])]


def test_acceptance_probe_runner_is_owned_by_tools_and_is_valid_shell():
    """可执行验收基础设施属于 tools，不应继续伪装成 pytest 测试。"""
    runner = ROOT / "tools" / "acceptance" / "run_probe.sh"
    source = runner.read_text(encoding="utf-8")

    assert not (ROOT / "tests/integration/run_probe.sh").exists()
    assert "PYTHONPATH" in source
    assert 'exec python3 -u "$PROBE" "$@"' in source
    subprocess.run(["bash", "-n", str(runner)], check=True)


def test_acceptance_probe_runner_preserves_args_exit_code_and_python_path(tmp_path):
    runner = ROOT / "tools" / "acceptance" / "run_probe.sh"
    probe = tmp_path / "record_probe.py"
    probe.write_text(
        """import json
import sys

print(json.dumps({
    "args": sys.argv[1:],
    "root_on_path": %r in sys.path,
    "existing_path_preserved": "/sentinel" in sys.path,
    "unbuffered": sys.stdout.write_through,
}))
raise SystemExit(17)
"""
        % str(ROOT),
        encoding="utf-8",
    )
    environment = dict(os.environ)
    environment["PYTHONPATH"] = "/sentinel"

    completed = subprocess.run(
        ["bash", str(runner), str(probe), "two words", "第二个参数"],
        cwd=tmp_path,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert completed.returncode == 17
    assert json.loads(completed.stdout) == {
        "args": ["two words", "第二个参数"],
        "root_on_path": True,
        "existing_path_preserved": True,
        "unbuffered": True,
    }

    # 相对路径必须基于仓库根，而不是调用者当前目录。
    relative = subprocess.run(
        ["bash", str(runner), "tests/repository/repository_test_support.py"],
        cwd=tmp_path,
        env=environment,
        check=False,
    )
    assert relative.returncode == 0


def test_literal_integration_probe_paths_resolve_to_tracked_files():
    """场景启动器不能继续引用重组前已经不存在的 probe 路径。"""
    missing: list[str] = []
    pattern = re.compile(
        r"(?<![A-Za-z0-9_.-])"
        r"((?:tests/integration|scripts|tools)/[A-Za-z0-9_./-]+\.(?:py|sh))"
    )
    scripts = list((ROOT / "scripts").glob("*.sh"))
    scripts.extend(
        ROOT / library for library in {mode.handler_library for mode in MODES}
    )
    scripts.append(ROOT / "tools/acceptance/handlers/common.sh")
    for script in sorted(scripts):
        source = script.read_text(encoding="utf-8")
        for relative_path in pattern.findall(source):
            if not (ROOT / relative_path).is_file():
                missing.append(f"{script.name}: {relative_path}")

    assert missing == []


def test_cli_dispatches_arguments_through_injected_runner():
    runner = RecordingRunner(result=17)

    status = main(
        ["continuous-nav2-evidence", "offline"],
        runner=runner,
        stdout=io.StringIO(),
        stderr=io.StringIO(),
    )

    assert status == 17
    assert runner.calls == [("continuous-nav2-evidence", ["offline"])]


def test_bash_runner_exposes_only_user_arguments_to_handler(tmp_path):
    """mode 名属于 Python 路由元数据，不能泄漏成 Bash handler 的隐藏 `$1`。"""
    mode = AcceptanceMode(
        name="sample-mode",
        description="test fixture",
        handler="accept_sample_mode",
        category="internal",
        public=False,
        requires_ros_environment=False,
        domain=HandlerDomain.CONTROL,
    )
    common = tmp_path / "tools/acceptance/handlers/common.sh"
    common.parent.mkdir(parents=True)
    common.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    (tmp_path / mode.handler_library).write_text(
        "accept_sample_mode() { printf '%s\\n' \"$#\" \"$@\" > \"$WORKSPACE/arguments.txt\"; }\n",
        encoding="utf-8",
    )

    status = BashModeRunner(tmp_path).run(
        mode,
        ["offline", "two words", "$(touch must-not-run)"],
    )

    assert status == 0
    assert (tmp_path / "arguments.txt").read_text(encoding="utf-8").splitlines() == [
        "3",
        "offline",
        "two words",
        "$(touch must-not-run)",
    ]
    assert not (tmp_path / "must-not-run").exists()


def test_cli_help_and_unknown_mode_do_not_execute_runner():
    runner = RecordingRunner()
    stdout = io.StringIO()
    stderr = io.StringIO()

    assert main([], runner=runner, stdout=stdout, stderr=stderr) == 0
    assert "Stable public modes:" in stdout.getvalue()
    assert "openloris-replay-stage" not in stdout.getvalue()

    assert main(["missing"], runner=runner, stdout=stdout, stderr=stderr) == 2
    assert "Unknown acceptance mode" in stderr.getvalue()
    assert runner.calls == []
