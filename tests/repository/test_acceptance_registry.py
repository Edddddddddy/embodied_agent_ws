from __future__ import annotations

import io
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

from tools.acceptance.catalog import (
    MODE_BY_NAME,
    MODES,
    PUBLIC_MODE_NAMES,
    AcceptanceMode,
)
from tools.acceptance.cli import main


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
        "robotics-gate",
    )
    assert {mode.name for mode in MODES if mode.public} == set(PUBLIC_MODE_NAMES)


def test_every_registered_mode_has_one_shell_handler():
    implemented: set[str] = set()
    for library in {mode.handler_library for mode in MODES}:
        source = (ROOT / library).read_text(encoding="utf-8")
        implemented.update(
            re.findall(r"^(accept_[a-z0-9_]+)\(\) \{", source, re.M)
        )

    assert len(MODE_BY_NAME) == len(MODES)
    assert {mode.handler for mode in MODES} == implemented


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
