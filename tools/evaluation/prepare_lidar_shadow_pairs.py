#!/usr/bin/env python3
"""Convert ranked loop-candidate JSONL into the explicit C++ shadow pair contract."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def load_odometry_priors(path: Path) -> dict[int, tuple[float, float, float]]:
    priors: dict[int, tuple[float, float, float]] = {}
    for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        if len(fields) != 6 or fields[0] != "O":
            raise ValueError(f"invalid odometry prior at line {line_number}")
        scan_id = int(fields[1])
        values = tuple(float(value) for value in fields[3:6])
        if scan_id in priors or not all(math.isfinite(value) for value in values):
            raise ValueError(f"invalid odometry prior id/value at line {line_number}")
        priors[scan_id] = values
    if not priors:
        raise ValueError("odometry prior file is empty")
    return priors


def _relative_pose(
    query: tuple[float, float, float], candidate: tuple[float, float, float]
) -> tuple[float, float, float]:
    world_x = candidate[0] - query[0]
    world_y = candidate[1] - query[1]
    cosine = math.cos(query[2])
    sine = math.sin(query[2])
    return (
        cosine * world_x + sine * world_y,
        -sine * world_x + cosine * world_y,
        math.atan2(math.sin(candidate[2] - query[2]), math.cos(candidate[2] - query[2])),
    )


def prepare(
    candidate_path: Path,
    output_path: Path,
    top_k: int,
    odometry_path: Path | None = None,
    corpus_metadata_path: Path | None = None,
    candidate_evidence_path: Path | None = None,
) -> dict[str, int | bool]:
    if top_k <= 0:
        raise ValueError("top-k must be positive")
    lines = ["# embodied_lidar_candidate_pairs_v1"]
    query_rows = 0
    pair_count = 0
    seen_pairs: set[tuple[int, int]] = set()
    priors = load_odometry_priors(odometry_path) if odometry_path else None
    source_contract_verified = False
    if (corpus_metadata_path is None) != (candidate_evidence_path is None):
        raise ValueError("corpus metadata and candidate evidence must be provided together")
    if corpus_metadata_path is not None and candidate_evidence_path is not None:
        metadata = json.loads(corpus_metadata_path.read_text(encoding="utf-8"))
        evidence = json.loads(candidate_evidence_path.read_text(encoding="utf-8"))
        expected = str(evidence["source"]["corpus_sha256"])
        actual = str(metadata["corpus"]["sha256"])
        if actual != expected:
            raise ValueError("candidate evidence and scan corpus SHA256 do not match")
        source_contract_verified = True
    for line_number, raw in enumerate(
        candidate_path.read_text(encoding="utf-8").splitlines(), 1
    ):
        if not raw.strip():
            continue
        row = json.loads(raw)
        query_id = int(row["query_id"])
        query_stamp = float(row["query_stamp_s"])
        if not math.isfinite(query_stamp):
            raise ValueError(f"non-finite query stamp at line {line_number}")
        query_rows += 1
        candidates = sorted(row["candidates"], key=lambda item: int(item["rank"]))
        for candidate in candidates[:top_k]:
            candidate_id = int(candidate["scan_id"])
            key = (query_id, candidate_id)
            if key in seen_pairs:
                raise ValueError(f"duplicate candidate pair at line {line_number}: {key}")
            seen_pairs.add(key)
            values = (
                float(candidate["stamp_s"]),
                float(candidate["yaw_offset_rad"]),
                float(candidate["similarity"]),
                float(candidate["ring_key_distance"]),
            )
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"non-finite candidate field at line {line_number}")
            prior_fields = ""
            if priors is not None:
                if query_id not in priors or candidate_id not in priors:
                    raise ValueError(f"candidate pair lacks odometry prior: {key}")
                relative = _relative_pose(priors[query_id], priors[candidate_id])
                prior_fields = " " + " ".join(f"{value:.12g}" for value in relative)
            lines.append(
                "P "
                f"{query_id} {candidate_id} {query_stamp:.12g} {values[0]:.12g} "
                f"{int(candidate['rank'])} {values[1]:.12g} {values[2]:.12g} "
                f"{values[3]:.12g}{prior_fields}"
            )
            pair_count += 1
    if query_rows == 0 or pair_count == 0:
        raise ValueError("candidate file did not contain any shadow pairs")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return {
        "query_rows": query_rows,
        "pairs": pair_count,
        "top_k": top_k,
        "odometry_prior": priors is not None,
        "source_contract_verified": source_contract_verified,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidates", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--top-k", type=int, default=10)
    parser.add_argument("--odometry-priors", type=Path)
    parser.add_argument("--corpus-metadata", type=Path)
    parser.add_argument("--candidate-evidence", type=Path)
    args = parser.parse_args()
    result = prepare(
        args.candidates,
        args.output,
        args.top_k,
        args.odometry_priors,
        args.corpus_metadata,
        args.candidate_evidence,
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print("PASS: ranked candidates -> shadow scan-match pair contract")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
