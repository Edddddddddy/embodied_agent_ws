"""仓库契约测试共享的只读路径与 Python 常量解析工具。"""

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
CORE_ROOT = ROOT / "src" / "embodied_agent_core" / "embodied_agent_core"
VOICE_FRONTEND_ROOT = ROOT / "src" / "embodied_voice_frontend" / "embodied_voice_frontend"


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
