import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "generate_architecture_facts.py"
SPEC = importlib.util.spec_from_file_location("generate_architecture_facts", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_architecture_contracts_are_derived_from_current_repository():
    facts = MODULE.build_facts(ROOT)

    assert facts["ros_packages"]["count"] == 12
    assert facts["ros_packages"]["names"] == facts["ros_packages"]["ci_names"]
    assert facts["acceptance_cli"]["public_mode_count"] == 7
    assert all(facts["contracts"].values())
    assert all(facts["release_gate"]["robotics_coverage"].values())
    assert facts["ci"]["feature_push_deduplicated"] is True


def test_committed_architecture_evidence_matches_generated_facts():
    facts = MODULE.build_facts(ROOT)
    committed_json = json.loads(
        (ROOT / "docs" / "evidence" / "architecture_facts.json").read_text(
            encoding="utf-8"
        )
    )
    committed_markdown = (
        ROOT / "docs" / "evidence" / "architecture_facts.md"
    ).read_text(encoding="utf-8")

    assert committed_json == facts
    assert committed_markdown == MODULE.render_markdown(facts)


def test_architecture_docs_delegate_volatile_counts_to_generated_evidence():
    audit = (ROOT / "docs" / "ARCHITECTURE_AUDIT.md").read_text(encoding="utf-8")
    gaps = (ROOT / "docs" / "PROJECT_GAPS_AND_OPTIMIZATION.md").read_text(
        encoding="utf-8"
    )

    assert "evidence/architecture_facts.md" in audit
    assert "docs/evidence/architecture_facts.md" in gaps
    for stale_claim in ("9 个 ROS 2 包", "约 90 个模式", "700～900 行"):
        assert stale_claim not in audit
    assert "当前 543/676 行" not in gaps
