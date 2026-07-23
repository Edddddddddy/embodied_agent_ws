import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "loop_frontend_trace", ROOT / "tools" / "evaluation" / "analyze_loop_frontend_trace.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC and SPEC.loader
SPEC.loader.exec_module(MODULE)


def _write(path: Path, events: list[dict]) -> None:
    path.write_text(
        "\n".join(json.dumps(event) for event in events) + "\n", encoding="utf-8"
    )


def test_reports_candidate_generation_boundary(tmp_path: Path) -> None:
    trace = tmp_path / "frontend.jsonl"
    _write(
        trace,
        [
            {
                "schema_version": 1,
                "event": "candidate_topology",
                "topology": {
                    "primary_reason": "all_geometric_neighbors_near_linked",
                    "historical_scan_count": 20,
                    "geometric_near_count": 5,
                    "near_linked_count": 5,
                },
                "replicated_karto_rule": {"candidate_chains": 0},
            }
        ],
    )
    report = MODULE.analyze_trace(trace)
    assert report["passed"] is True
    assert report["failure_boundary"] == "candidate_generation"
    assert report["candidate_primary_reasons"] == {
        "all_geometric_neighbors_near_linked": 1
    }


def test_reports_coarse_and_accepted_loop_stages(tmp_path: Path) -> None:
    trace = tmp_path / "frontend.jsonl"
    _write(
        trace,
        [
            {
                "schema_version": 1,
                "event": "candidate_topology",
                "topology": {"primary_reason": "matcher_candidate_available"},
                "replicated_karto_rule": {"candidate_chains": 1},
            },
            {
                "schema_version": 1,
                "event": "matcher_coarse_check",
                "response": 0.5,
                "response_threshold": 0.3,
                "variance_x": 1.0,
                "variance_y": 2.0,
                "variance_threshold": 3.0,
            },
            {
                "schema_version": 1,
                "event": "matcher_fine_check",
                "response": 0.6,
                "response_threshold": 0.4,
            },
            {"schema_version": 1, "event": "begin_closure"},
            {"schema_version": 1, "event": "end_closure"},
        ],
    )
    report = MODULE.analyze_trace(trace)
    assert report["failure_boundary"] == "accepted_loop"
    assert report["events"]["coarse_passed"] == 1
    assert report["events"]["fine_passed"] == 1


def test_rejects_malformed_evidence(tmp_path: Path) -> None:
    trace = tmp_path / "frontend.jsonl"
    trace.write_text("not-json\n", encoding="utf-8")
    report = MODULE.analyze_trace(trace)
    assert report["passed"] is False
    assert report["checks"]["all_lines_parseable"] is False
