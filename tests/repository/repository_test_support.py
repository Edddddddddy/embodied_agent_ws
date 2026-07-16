"""仓库契约测试共享的只读路径与 Python 常量解析工具。"""

import ast
from pathlib import Path

from tools.acceptance.catalog import MODE_BY_NAME


ROOT = Path(__file__).resolve().parents[2]
CORE_ROOT = ROOT / "src" / "embodied_agent_core" / "embodied_agent_core"
BRINGUP_ROOT = ROOT / "src" / "embodied_agent_bringup" / "embodied_agent_bringup"
VOICE_FRONTEND_ROOT = ROOT / "src" / "embodied_voice_frontend" / "embodied_voice_frontend"


def assert_acceptance_modes(*names: str) -> None:
    """Validate the runtime registry instead of grepping the thin shell adapter."""

    missing = sorted(set(names) - MODE_BY_NAME.keys())
    assert not missing, f"acceptance modes missing from registry: {missing}"


def acceptance_handler_source(name: str) -> str:
    """Return one registered shell adapter body for a narrow implementation check."""

    mode = MODE_BY_NAME[name]
    source = (ROOT / mode.handler_library).read_text(encoding="utf-8")
    marker = f"\n{mode.handler}() {{\n"
    start = source.find(marker)
    assert start >= 0, f"handler not found: {mode.handler}"
    next_handler = source.find("\naccept_", start + len(marker))
    return source[start : next_handler if next_handler >= 0 else len(source)]


def _python_literal(module_path: Path, name: str):
    tree = ast.parse(module_path.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            if node.target.id == name:
                return ast.literal_eval(node.value)
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == name:
                    return ast.literal_eval(node.value)
    raise AssertionError(f"{name} not found in {module_path}")
