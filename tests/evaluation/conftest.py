"""Make repository-local Agent packages available to subprocess evaluation tests."""

from __future__ import annotations

import os
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SOURCE_DIRS = (
    ROOT / "src" / "embodied_agent_core",
    ROOT / "src" / "embodied_online_agent",
    ROOT / "src" / "embodied_offline_agent",
)
for source in reversed(SOURCE_DIRS):
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))

existing = os.environ.get("PYTHONPATH", "")
os.environ["PYTHONPATH"] = os.pathsep.join(
    [*(str(source) for source in SOURCE_DIRS), *([existing] if existing else [])]
)
