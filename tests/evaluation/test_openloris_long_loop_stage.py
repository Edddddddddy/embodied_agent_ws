from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_long_loop_stage_is_pinned_and_preserves_evidence_boundaries():
    stage = (ROOT / "tools" / "evaluation" / "run_openloris_long_loop_evidence.sh").read_text(
        encoding="utf-8"
    )
    setup = (ROOT / "tools" / "evaluation" / "setup_openloris_rosbag.py").read_text(
        encoding="utf-8"
    )
    replay = (ROOT / "tools" / "evaluation" / "run_openloris_slam_replay.sh").read_text(
        encoding="utf-8"
    )
    review = (ROOT / "tools" / "evaluation" / "extract_openloris_review_frames.py").read_text(
        encoding="utf-8"
    )
    compact = (ROOT / "tools" / "evaluation" / "compact_openloris_rosbag.py").read_text(
        encoding="utf-8"
    )

    assert 'SEQUENCE="${OPENLORIS_SEQUENCE:-corridor1-1}"' in stage
    assert "corridor1-1|corridor1-2" in stage
    assert '"duration_at_least_90s"' in stage
    assert "rank_openloris_revisit_sequences.py" in stage
    assert "--verify-source-hash" in stage
    assert "OPENLORIS_EVALUATE_LOOP_CONSTRAINTS=true" in stage
    assert "OPENLORIS_EVALUATE_FRONTEND=true" in stage
    assert "openloris_corridor1_1_annotations.json" in stage
    assert 'OPENLORIS_ANNOTATIONS="$ANNOTATIONS"' in stage
    assert 'SLAM_LOOP_YAW_TOLERANCE_DEG="${SLAM_LOOP_YAW_TOLERANCE_DEG:-180.0}"' in stage
    assert 'SLAM_LOOP_MIN_SEPARATION_S="${SLAM_LOOP_MIN_SEPARATION_S:-60.0}"' in stage
    assert "a373fb24539561ee6a8900c91603baeedf8b739881a7b04860ed5e363dc93a22" in setup
    assert "e8a25490762e8b07537a4297cf6b228b8eee42345587cb3892776264995eb3b6" in setup
    assert '--loop-yaw-tolerance-deg "${SLAM_LOOP_YAW_TOLERANCE_DEG:-30.0}"' in replay
    assert "start=target_stamp_ns" in review
    assert "writer.write(outputs[connection.id], timestamp_ns, rawdata)" in compact
