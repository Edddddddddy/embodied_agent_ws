from __future__ import annotations

import pytest

from tools.acceptance.dynamic_route import (
    ReplanRouteRejected,
    select_replannable_route,
)


def test_selector_recovers_after_dynamic_path_failure_and_uses_fallback() -> None:
    attempted: list[str] = []
    recovered: list[str] = []

    def attempt(goal: dict) -> dict:
        attempted.append(goal["name"])
        if goal["name"] == "meeting_room":
            raise ReplanRouteRejected("dynamic ComputePathToPose error=208")
        return {"goal": goal["name"], "dynamic_path": "reachable"}

    selected, audit = select_replannable_route(
        [{"name": "meeting_room"}, {"name": "entrance"}],
        attempt=attempt,
        recover=lambda goal, _error: recovered.append(goal["name"]),
    )

    assert selected["goal"] == "entrance"
    assert attempted == ["meeting_room", "entrance"]
    assert recovered == ["meeting_room"]
    assert audit == [
        {
            "goal": "meeting_room",
            "accepted": False,
            "error": "dynamic ComputePathToPose error=208",
        },
        {"goal": "entrance", "accepted": True, "error": ""},
    ]


def test_selector_reports_every_rejected_candidate() -> None:
    with pytest.raises(ReplanRouteRejected, match="meeting_room.*entrance"):
        select_replannable_route(
            [{"name": "meeting_room"}, {"name": "entrance"}],
            attempt=lambda goal: (_ for _ in ()).throw(
                ReplanRouteRejected(f"{goal['name']} blocked")
            ),
            recover=lambda _goal, _error: None,
        )
