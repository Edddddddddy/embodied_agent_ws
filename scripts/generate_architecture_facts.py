#!/usr/bin/env python3
"""Generate deterministic architecture facts from the repository itself.

文档中的包数量、验收模式数量和节点行数很容易随迭代失真。这个工具不读取人工维护的
“结论”，而是从 package.xml、CI workflow、验收路由和源码重新计算事实。提交的 JSON/Markdown
快照由 repository tests 校验，后续结构变化必须显式刷新证据。
"""

from __future__ import annotations

import argparse
import json
import re
import runpy
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


DEFAULT_JSON = Path("docs/evidence/architecture_facts.json")
DEFAULT_MARKDOWN = Path("docs/evidence/architecture_facts.md")


def _line_count(path: Path) -> int:
    return len(path.read_text(encoding="utf-8").splitlines())


def _ros_packages(root: Path) -> list[str]:
    packages: list[str] = []
    for manifest in sorted((root / "src").glob("*/package.xml")):
        name = ET.parse(manifest).getroot().findtext("name")
        if name:
            packages.append(name.strip())
    return sorted(packages)


def _ci_packages(workflow: str) -> list[str]:
    """Parse action-ros-ci's folded package-name block without a YAML dependency."""

    lines = workflow.splitlines()
    for index, line in enumerate(lines):
        if line.strip() != "package-name: >-":
            continue
        base_indent = len(line) - len(line.lstrip())
        values: list[str] = []
        for candidate in lines[index + 1 :]:
            if not candidate.strip():
                continue
            indent = len(candidate) - len(candidate.lstrip())
            if indent <= base_indent:
                break
            values.extend(candidate.split())
        return sorted(values)
    raise ValueError("CI workflow does not contain an action-ros-ci package-name block")


def _acceptance_registry(root: Path) -> tuple[list[str], list[str], list[str]]:
    """Read CLI facts from the same registry used at runtime.

    旧实现反向解析 1000 行 shell ``case`` 和 help heredoc，重排空格就会误报。
    现在 Python 注册表是唯一事实源，架构审计直接验证它的公开/内部边界。
    """

    root_text = str(root)
    if root_text not in sys.path:
        sys.path.insert(0, root_text)
    from tools.acceptance.catalog import MODES, PUBLIC_MODE_NAMES

    public_modes = sorted(PUBLIC_MODE_NAMES)
    all_modes = sorted(mode.name for mode in MODES)
    return public_modes, all_modes, all_modes


def _release_profiles(root: Path) -> dict[str, list[dict[str, str]]]:
    namespace = runpy.run_path(str(root / "scripts" / "showcase_release_gate.py"))
    profiles: dict[str, list[dict[str, str]]] = {}
    for profile, commands in sorted(namespace["PROFILE_COMMANDS"].items()):
        profiles[profile] = [
            {"name": str(name), "command": str(command)} for name, command in commands
        ]
    return profiles


def _interface_inventory(root: Path) -> dict[str, Any]:
    interface_root = root / "src" / "embodied_agent_interfaces"
    inventory: dict[str, Any] = {}
    for kind, directory, suffix in (
        ("messages", "msg", "*.msg"),
        ("services", "srv", "*.srv"),
        ("actions", "action", "*.action"),
    ):
        files = sorted(path.name for path in (interface_root / directory).glob(suffix))
        inventory[kind] = {"count": len(files), "files": files}
    inventory["total"] = sum(inventory[kind]["count"] for kind in ("messages", "services", "actions"))
    return inventory


def _source_files(root: Path) -> list[Path]:
    """Ignore interpreter caches so running the audit cannot change its own result."""

    return sorted(
        path
        for path in root.rglob("*")
        if path.is_file()
        and "__pycache__" not in path.parts
        and path.suffix not in {".pyc", ".pyo"}
    )


def build_facts(root: Path) -> dict[str, Any]:
    root = root.resolve()
    workflow = (root / ".github" / "workflows" / "ros2-ci.yml").read_text(
        encoding="utf-8"
    )
    handler_paths = sorted((root / "tools" / "acceptance" / "handlers").glob("*.sh"))
    handler_paths.append(root / "tools" / "evaluation" / "acceptance_handlers.sh")
    acceptance_handlers = "\n".join(
        path.read_text(encoding="utf-8") for path in handler_paths
    )

    ros_packages = _ros_packages(root)
    ci_packages = _ci_packages(workflow)
    public_modes, advanced_modes, router_modes = _acceptance_registry(root)
    release_profiles = _release_profiles(root)
    robotics_text = "\n".join(
        command["command"] for command in release_profiles["robotics"]
    )

    robotics_coverage = {
        "architecture_facts": "architecture-facts" in robotics_text,
        "repository_and_agent_units": all(
            token in robotics_text
            for token in (
                "tests/repository",
                "src/embodied_online_agent/test",
                "src/embodied_offline_agent/test",
            )
        ),
        "required_cpp_packages": all(
            package in robotics_text
            for package in (
                "embodied_agent_bringup",
                "embodied_agent_cpp",
                "embodied_simulation",
                "embodied_navigation",
                "embodied_slam",
            )
        ),
        "continuous_multi_command": "continuous-multi-command" in robotics_text,
        "nav2_stage": "nav2-stage" in robotics_text,
        "slam_evaluation": "slam-evaluation-stage" in robotics_text,
        "openloris_fixture": "openloris-replay-stage" in robotics_text,
        "dynamic_obstacle": "dynamic-obstacle-stage" in robotics_text,
    }

    trigger_lines = workflow.count("branches: [main, dev]")
    release_gate_workspace_is_explicit = all(
        re.search(
            rf"accept_{mode.replace('-', '_')}\(\)\s*\{{.*?"
            rf"showcase_release_gate\.py.*?--workspace \"\$WORKSPACE\"",
            acceptance_handlers,
            re.DOTALL,
        )
        is not None
        for mode in ("release-gate", "robotics-gate", "demo-gate")
    )
    contracts = {
        "ci_matrix_matches_ros_packages": ci_packages == ros_packages,
        "ci_push_and_pr_only_dev_main": trigger_lines == 2,
        "public_mode_count_is_8": len(public_modes) == 8,
        "shell_entry_is_thin": _line_count(root / "scripts" / "acceptance_test.sh") <= 15,
        "public_modes_are_routable": set(public_modes) <= set(router_modes),
        "advanced_modes_are_routable": set(advanced_modes) <= set(router_modes),
        "release_gates_use_current_workspace": release_gate_workspace_is_explicit,
        "robotics_profile_covers_required_gates": all(robotics_coverage.values()),
    }

    online_node = root / "src" / "embodied_online_agent" / "embodied_online_agent" / "online_agent_node.py"
    offline_node = root / "src" / "embodied_offline_agent" / "embodied_offline_agent" / "offline_agent_node.py"
    scripts_root = root / "scripts"
    script_files = _source_files(scripts_root)

    return {
        "schema_version": 1,
        "source": "repository_deterministic_scan",
        "contracts": contracts,
        "ros_packages": {
            "count": len(ros_packages),
            "names": ros_packages,
            "ci_count": len(ci_packages),
            "ci_names": ci_packages,
        },
        "acceptance_cli": {
            "public_mode_count": len(public_modes),
            "public_modes": public_modes,
            "advanced_help_mode_count": len(advanced_modes),
            "router_mode_count": len(router_modes),
            "undocumented_router_modes": sorted(set(router_modes) - set(advanced_modes) - set(public_modes)),
        },
        "repository_scale": {
            "scripts_top_level_file_count": sum(1 for path in script_files if path.parent == scripts_root),
            "scripts_recursive_file_count": len(script_files),
            "online_agent_node_lines": _line_count(online_node),
            "offline_agent_node_lines": _line_count(offline_node),
        },
        "typed_interfaces": _interface_inventory(root),
        "ci": {
            "workflow": ".github/workflows/ros2-ci.yml",
            "push_branches": ["main", "dev"],
            "pull_request_branches": ["main", "dev"],
            "feature_push_deduplicated": trigger_lines == 2,
        },
        "release_gate": {
            "profiles": release_profiles,
            "robotics_coverage": robotics_coverage,
        },
        "evidence_boundary": (
            "本报告只证明仓库结构、CI/CLI 契约和静态覆盖关系；不证明真实麦克风成功率、"
            "模型精度、Gazebo 物理运动或真实场景 SLAM 泛化。"
        ),
    }


def render_markdown(facts: dict[str, Any]) -> str:
    packages = facts["ros_packages"]
    cli = facts["acceptance_cli"]
    scale = facts["repository_scale"]
    interfaces = facts["typed_interfaces"]
    contracts = facts["contracts"]
    robotics = facts["release_gate"]["robotics_coverage"]

    lines = [
        "# 架构事实报告",
        "",
        "> 本文件由 `scripts/generate_architecture_facts.py` 从仓库确定性生成；不要手工修改。",
        "",
        "## 当前规模",
        "",
        "| 项目 | 当前事实 |",
        "| --- | ---: |",
        f"| ROS 2 package | {packages['count']} |",
        f"| GitHub Actions 构建 package | {packages['ci_count']} |",
        f"| 公开验收入口 | {cli['public_mode_count']} |",
        f"| 高级帮助入口 | {cli['advanced_help_mode_count']} |",
        f"| CLI router mode | {cli['router_mode_count']} |",
        f"| `scripts/` 顶层文件 | {scale['scripts_top_level_file_count']} |",
        f"| `scripts/` 递归文件 | {scale['scripts_recursive_file_count']} |",
        f"| online Agent 主节点行数 | {scale['online_agent_node_lines']} |",
        f"| offline Agent 主节点行数 | {scale['offline_agent_node_lines']} |",
        f"| 自定义 msg/srv/action 总数 | {interfaces['total']} |",
        "",
        "## ROS 2 package 与 CI 矩阵",
        "",
        ", ".join(f"`{name}`" for name in packages["names"]),
        "",
        "## 公开验收入口",
        "",
        ", ".join(f"`{mode}`" for mode in cli["public_modes"]),
        "",
        "## 静态契约",
        "",
    ]
    lines.extend(
        f"- {'PASS' if value else 'FAIL'}：`{name}`" for name, value in contracts.items()
    )
    lines.extend(["", "## Robotics release gate 覆盖", ""])
    lines.extend(
        f"- {'PASS' if value else 'FAIL'}：`{name}`" for name, value in robotics.items()
    )
    lines.extend(
        [
            "",
            "## 证据边界",
            "",
            facts["evidence_boundary"],
            "",
            "重新生成：",
            "",
            "```bash",
            "python3 scripts/generate_architecture_facts.py",
            "bash scripts/acceptance_test.sh architecture-facts",
            "```",
            "",
        ]
    )
    return "\n".join(lines)


def _resolved_output(root: Path, value: str, default: Path) -> Path:
    output = Path(value) if value else default
    return output if output.is_absolute() else root / output


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", default=".")
    parser.add_argument("--json-output", default="")
    parser.add_argument("--markdown-output", default="")
    parser.add_argument(
        "--check",
        action="store_true",
        help="Compare committed reports with current repository facts without rewriting them.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    root = Path(args.workspace).expanduser().resolve()
    json_output = _resolved_output(root, args.json_output, DEFAULT_JSON)
    markdown_output = _resolved_output(root, args.markdown_output, DEFAULT_MARKDOWN)
    facts = build_facts(root)
    json_text = json.dumps(facts, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    markdown_text = render_markdown(facts)

    failed_contracts = [name for name, passed in facts["contracts"].items() if not passed]
    if failed_contracts:
        print("FAIL: architecture contracts: " + ", ".join(failed_contracts), file=sys.stderr)
        raise SystemExit(1)

    if args.check:
        mismatches = []
        for path, expected in ((json_output, json_text), (markdown_output, markdown_text)):
            if not path.is_file() or path.read_text(encoding="utf-8") != expected:
                mismatches.append(str(path.relative_to(root)))
        if mismatches:
            print(
                "FAIL: stale architecture facts: " + ", ".join(mismatches),
                file=sys.stderr,
            )
            print("Run: python3 scripts/generate_architecture_facts.py", file=sys.stderr)
            raise SystemExit(1)
        print("PASS: committed architecture facts match the repository")
        return

    json_output.parent.mkdir(parents=True, exist_ok=True)
    markdown_output.parent.mkdir(parents=True, exist_ok=True)
    json_output.write_text(json_text, encoding="utf-8")
    markdown_output.write_text(markdown_text, encoding="utf-8")
    print(json_output)
    print(markdown_output)


if __name__ == "__main__":
    main()
