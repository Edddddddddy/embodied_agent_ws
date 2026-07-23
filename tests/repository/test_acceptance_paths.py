"""验收工具路径定位契约。"""

from __future__ import annotations

from pathlib import Path

import pytest

from repository_test_support import ROOT
from tools.acceptance.paths import repository_root


def test_repository_root_is_independent_from_probe_directory_depth():
    nested_probe = ROOT / "tools/acceptance/probes/slam_nav/example.py"

    assert repository_root(nested_probe) == ROOT


def test_repository_root_supports_an_exported_or_worktree_layout(tmp_path):
    (tmp_path / "scripts").mkdir()
    (tmp_path / "scripts/acceptance_test.sh").touch()
    (tmp_path / "tools/acceptance").mkdir(parents=True)
    (tmp_path / "src").mkdir()
    nested = tmp_path / "tools/acceptance/probes/slam_nav"
    nested.mkdir(parents=True)

    assert repository_root(nested) == tmp_path


def test_repository_root_fails_explicitly_outside_a_workspace(tmp_path):
    with pytest.raises(FileNotFoundError, match="repository root not found"):
        repository_root(tmp_path)


def test_path_interface_stays_ros_free():
    source = (ROOT / "tools/acceptance/paths.py").read_text(encoding="utf-8")

    assert "rclpy" not in source
    assert "ament_index" not in source
