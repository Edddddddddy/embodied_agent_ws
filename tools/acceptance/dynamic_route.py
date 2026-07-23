"""Policy for selecting a route that remains reachable after obstacle injection."""

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping
from typing import TypeVar


Candidate = Mapping[str, object]
Selected = TypeVar("Selected")


class ReplanRouteRejected(RuntimeError):
    """A candidate is valid statically but unsuitable for dynamic replanning."""


def select_replannable_route(
    candidates: Iterable[Candidate],
    *,
    attempt: Callable[[Candidate], Selected],
    recover: Callable[[Candidate, ReplanRouteRejected], None],
) -> tuple[Selected, list[dict[str, object]]]:
    """Return the first end-to-end replannable candidate and an audit trail.

    ``attempt`` must cover both baseline planning and planning with the predicted
    obstacle active.  A rejected attempt is recovered before the next candidate,
    so stale tracks/costmap cells cannot leak across trials.
    """

    audit: list[dict[str, object]] = []
    failures: list[str] = []
    for candidate in candidates:
        name = str(candidate.get("name") or "unnamed")
        try:
            selected = attempt(candidate)
        except ReplanRouteRejected as error:
            audit.append({"goal": name, "accepted": False, "error": str(error)})
            failures.append(f"{name}: {error}")
            recover(candidate, error)
            continue
        audit.append({"goal": name, "accepted": True, "error": ""})
        return selected, audit

    detail = " | ".join(failures) or "no candidate goals"
    raise ReplanRouteRejected(f"no dynamically replannable route: {detail}")
