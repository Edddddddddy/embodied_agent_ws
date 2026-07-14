#!/usr/bin/env python3
"""Materialize controlled slam_toolbox configs for an accepted-edge loop sweep."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import re
from pathlib import Path

import yaml


ALLOWED_OVERRIDES = {
    "loop_match_minimum_chain_size",
    "loop_match_maximum_variance_coarse",
    "loop_match_minimum_response_coarse",
    "loop_match_minimum_response_fine",
    "loop_search_maximum_distance",
    "loop_search_space_dimension",
    "loop_search_space_resolution",
    "loop_search_space_smear_deviation",
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build_configs(base: Path, matrix: Path, output_dir: Path) -> dict[str, object]:
    source = yaml.safe_load(base.read_text(encoding="utf-8"))
    definition = json.loads(matrix.read_text(encoding="utf-8"))
    parameters = source.get("slam_toolbox", {}).get("ros__parameters", {})
    if parameters.get("solver_plugin") != "embodied_slam::GtsamScanSolver":
        raise ValueError("loop sweep requires the GTSAM production config as its base")
    profiles = definition.get("profiles")
    if not isinstance(profiles, list) or not profiles:
        raise ValueError("matrix must contain at least one profile")

    output_dir.mkdir(parents=True, exist_ok=True)
    generated: list[dict[str, object]] = []
    seen: set[str] = set()
    for profile in profiles:
        profile_id = str(profile.get("id", ""))
        if not re.fullmatch(r"[a-z][a-z0-9_]*", profile_id) or profile_id in seen:
            raise ValueError(f"invalid or duplicate profile id: {profile_id!r}")
        seen.add(profile_id)
        overrides = profile.get("overrides", {})
        if not isinstance(overrides, dict):
            raise ValueError(f"{profile_id}: overrides must be an object")
        unsupported = set(overrides) - ALLOWED_OVERRIDES
        if unsupported:
            raise ValueError(f"{profile_id}: unsupported overrides: {sorted(unsupported)}")
        missing = set(overrides) - set(parameters)
        if missing:
            raise ValueError(f"{profile_id}: base config misses keys: {sorted(missing)}")

        rendered = copy.deepcopy(source)
        rendered_params = rendered["slam_toolbox"]["ros__parameters"]
        rendered_params.update(overrides)
        output = output_dir / f"{profile_id}.yaml"
        output.write_text(
            yaml.safe_dump(rendered, sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )
        generated.append(
            {
                "id": profile_id,
                "description": profile.get("description", ""),
                "overrides": overrides,
                "effective_values": {
                    key: rendered_params[key] for key in sorted(ALLOWED_OVERRIDES)
                },
                "path": str(output.resolve()),
                "sha256": sha256(output),
            }
        )

    return {
        "schema_version": 1,
        "base": {"path": str(base.resolve()), "sha256": sha256(base)},
        "matrix": {"path": str(matrix.resolve()), "sha256": sha256(matrix)},
        "profiles": generated,
        "controlled_variables": definition.get("controlled_variables", []),
        "claim_boundary": definition.get("claim_boundary"),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", type=Path, required=True)
    parser.add_argument("--matrix", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    args = parser.parse_args()
    report = build_configs(args.base, args.matrix, args.output_dir)
    args.manifest.parent.mkdir(parents=True, exist_ok=True)
    args.manifest.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"PASS: generated {len(report['profiles'])} OpenLORIS sweep configs")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
