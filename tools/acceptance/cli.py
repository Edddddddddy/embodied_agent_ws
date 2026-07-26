#!/usr/bin/env python3
"""Thin, testable command-line interface for project acceptance workflows."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Sequence, TextIO

if __package__ in {None, ""}:
    # Allow direct execution from a fresh clone without installing this tooling package.
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.acceptance.catalog import MODE_BY_NAME, MODES, PUBLIC_MODE_NAMES, AcceptanceMode
from tools.acceptance.runner import BashModeRunner, ModeRunner


def workspace_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _print_modes(title: str, modes: Sequence[AcceptanceMode], stream: TextIO) -> None:
    print(title, file=stream)
    width = max(len(mode.name) for mode in modes)
    for mode in modes:
        print(f"  {mode.name:<{width}}  {mode.description}", file=stream)


def print_public_help(stream: TextIO = sys.stdout) -> None:
    print("Usage: acceptance_test.sh MODE [ARG ...]\n", file=stream)
    _print_modes(
        "Stable public modes:",
        tuple(MODE_BY_NAME[name] for name in PUBLIC_MODE_NAMES),
        stream,
    )
    print(
        "\nExample: bash scripts/acceptance_test.sh verify voice",
        file=stream,
    )
    print("\nUse --help-all only when maintaining internal/evaluation workflows.", file=stream)


def print_all_help(stream: TextIO = sys.stdout) -> None:
    print("Usage: acceptance_test.sh MODE [ARG ...]\n", file=stream)
    for category, title in (
        ("public", "Stable public modes:"),
        ("interactive", "Additional interactive modes:"),
        ("internal", "Internal regression modes:"),
        ("evaluation", "Evaluation/experiment modes:"),
    ):
        category_modes = tuple(mode for mode in MODES if mode.category == category)
        if category_modes:
            _print_modes(title, category_modes, stream)
            print(file=stream)


def main(
    argv: Sequence[str] | None = None,
    *,
    runner: ModeRunner | None = None,
    stdout: TextIO = sys.stdout,
    stderr: TextIO = sys.stderr,
) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    if not arguments or arguments[0] in {"help", "--help", "-h"}:
        print_public_help(stdout)
        return 0
    if arguments[0] in {"help-all", "--help-all"}:
        print_all_help(stdout)
        return 0

    mode_name, *mode_arguments = arguments
    mode = MODE_BY_NAME.get(mode_name)
    if mode is None:
        print(f"Unknown acceptance mode: {mode_name}", file=stderr)
        print("Run `bash scripts/acceptance_test.sh --help` for stable modes.", file=stderr)
        return 2

    selected_runner = runner or BashModeRunner(workspace_root())
    return selected_runner.run(mode, mode_arguments)


if __name__ == "__main__":
    raise SystemExit(main())
