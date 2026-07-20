"""固定 Explore Lite 补丁的版本、接口与安全安装语义。"""

from __future__ import annotations

import re
import subprocess

import yaml

from repository_test_support import ROOT


PINNED_REVISION = "326cf8a0b487c34246bb8f3326afbcd69576dc60"
PATCHES = (
    ROOT / "patches/m-explore-ros2/0001-frontier-telemetry-tf.patch",
    ROOT / "patches/m-explore-ros2/0002-goal-ledger-cooldown.patch",
    ROOT / "patches/m-explore-ros2/0003-reachable-frontier-approach.patch",
    ROOT / "patches/m-explore-ros2/0004-successful-frontier-cooldown.patch",
    ROOT / "patches/m-explore-ros2/0005-frontier-approach-clearance.patch",
    ROOT / "patches/m-explore-ros2/0006-clearance-reachable-frontier-path.patch",
    ROOT / "patches/m-explore-ros2/0007-active-frontier-goal-ownership.patch",
    ROOT / "patches/m-explore-ros2/0008-goal-lifecycle-state-machine.patch",
    ROOT / "patches/m-explore-ros2/0009-frontier-cost-metric-units.patch",
    ROOT / "patches/m-explore-ros2/0010-frontier-attempt-memory.patch",
    ROOT / "patches/m-explore-ros2/0011-approach-reached-tolerance.patch",
    ROOT / "patches/m-explore-ros2/0012-execution-clearance-contract.patch",
    ROOT / "patches/m-explore-ros2/0013-frontier-progress-recovery-contract.patch",
    ROOT / "patches/m-explore-ros2/0014-observable-approach-escape-contract.patch",
)
SETUP = ROOT / "scripts/setup_frontier_exploration.sh"
CI_WORKFLOW = ROOT / ".github/workflows/ros2-ci.yml"
FRONTIER_CONFIG = ROOT / "src/embodied_simulation/config/frontier_exploration.yaml"
UNKNOWN_WORLD_MISSION = (
    ROOT / "src/embodied_simulation/config/unknown_world_slam_mission.yaml"
)


def test_frontier_patch_and_setup_share_the_pinned_revision():
    repos = yaml.safe_load(
        (ROOT / "config/frontier_exploration.repos").read_text(encoding="utf-8")
    )
    revision = repos["repositories"]["third_party/m-explore-ros2"]["version"]
    setup = SETUP.read_text(encoding="utf-8")

    assert revision == PINNED_REVISION
    assert f'REVISION="{PINNED_REVISION}"' in setup
    assert all(patch.is_file() for patch in PATCHES)


def test_ci_replays_and_compiles_patch_stack_from_clean_upstream():
    setup = SETUP.read_text(encoding="utf-8")
    workflow = CI_WORKFLOW.read_text(encoding="utf-8")
    workflow_config = yaml.safe_load(workflow)
    replay_job = workflow_config["jobs"]["frontier-patch-replay"]

    assert "--prepare-only" in setup
    assert "frontier-patch-replay:" in workflow
    assert "setup_frontier_exploration.sh --prepare-only" in workflow
    assert "Initialize rosdep metadata" in workflow
    assert "20-default.list" in workflow
    assert "rosdep init" in workflow
    assert "rosdep update" in workflow
    # GitHub 容器的默认 /bin/sh 不支持 source；契约测试防止 shell 配置被误删。
    assert replay_job["defaults"]["run"]["shell"] == "bash"
    assert "--base-paths third_party/m-explore-ros2" in workflow
    assert "ctest --test-dir build/explore_lite" in workflow
    assert "-R '^test_explore$' --output-on-failure" in workflow
    # 上游 lint 债务不应掩盖本项目补丁的功能 GTest 结果。
    assert "colcon test-result --test-result-base build/explore_lite" not in workflow


def test_frontier_patch_exposes_auditable_goal_lifecycle_and_tf_fix():
    patch = "\n".join(item.read_text(encoding="utf-8") for item in PATCHES)
    for field in (
        "detected_frontier_count",
        "available_frontier_count",
        "blacklisted_frontier_count",
        "active_goal_count",
        "active_goal_id",
        "accepted_goal_count",
        "succeeded_goal_count",
        "aborted_goal_count",
        "canceled_goal_count",
        "rejected_goal_count",
        "last_goal_terminal",
        "completion_reason",
    ):
        assert field in patch

    assert "EXPLORATION_BLOCKED=exploration_blocked" in patch
    assert "frontier_attempts_exhausted_recoverable" in patch
    assert "goal_rejection_backoff" in patch
    assert "max_goal_rejections" in patch
    assert "builtin_interfaces::msg::Time()" in patch
    # UUID ledger 同时结算被抢占的旧目标；只有当前 UUID 能改变执行状态。
    assert "terminal_goal_ids_" in patch
    assert "is_current_goal" in patch
    assert "goal_terminal_cooldown" in patch


def test_frontier_navigation_target_is_reachable_free_space_not_centroid():
    patch = PATCHES[2].read_text(encoding="utf-8")
    attempt_patch = PATCHES[9].read_text(encoding="utf-8")

    assert "geometry_msgs::msg::Point approach" in patch
    assert "lastSearchValid()" in patch
    assert "INSCRIBED_INFLATED_OBSTACLE" in patch
    assert "!is_traversable(pos)" in patch
    assert "is_traversable(nbr)" in patch
    assert "map_[nbr] != FREE_SPACE" in patch
    assert "reachable_flag[nbr]" in patch
    assert "target_identity = frontier->centroid" in patch
    assert "target_position = frontier->approach" in patch
    # 最终策略同时携带 frontier identity 与物理 approach；两者不能混成一个点。
    assert "FrontierGoalKey{candidate.centroid, candidate.approach}" in attempt_patch
    assert "frontier_attempt_memory_.isSelectable" in attempt_patch
    assert "navigationGoalUsesReachableFreeSpaceApproach" in patch
    assert "rejectsDisconnectedFreePocketWhenRobotCellIsNotFree" in patch
    assert "inflatedRobotCellStillFindsReachableFrontier" in patch
    assert "validEmptySearchIsDistinguishableFromInvalidStart" in patch

    # caller 必须先判定 search validity，合法 empty 才能进入 no_frontiers。
    invalid_guard = patch.index("if (!search_.lastSearchValid())")
    complete_guard = patch.index("if (frontiers.empty())")
    assert invalid_guard < complete_guard


def test_frontier_attempt_memory_replaces_ttl_cooldown_and_identity_blacklist():
    patch = PATCHES[9].read_text(encoding="utf-8")
    added = "\n".join(
        line[1:]
        for line in patch.splitlines()
        if line.startswith("+") and not line.startswith("+++")
    )
    removed = "\n".join(
        line[1:]
        for line in patch.splitlines()
        if line.startswith("-") and not line.startswith("---")
    )

    assert "class FrontierAttemptMemory" in added
    assert "struct FrontierGoalKey" in added
    assert "std::hypot" in added
    assert "recordSucceeded" in added
    assert "recordFailed" in added
    assert "eraseNear" in added
    assert "summarizeCurrent" in added
    assert "active_failed_identity_count" in added
    assert "active_failed_approach_count" in added
    assert "active_failed_candidate_count" in added
    assert "frontier_attempt_memory_.isSelectable" in added
    assert "frontier_attempt_memory_.resetEpoch()" in added
    assert "lateSuccessSupersedesFailureMemory" in added
    assert "disappearedFailureIsNotCountedAsActive" in added
    assert "summaryClassifiesAllSuppressedCandidates" in added

    # 时间推进不能复活尝试；全部候选耗尽只能进入 recoverable blocked，
    # 不能继续沿用 cooldown 伪装成等待，也不能把 unknown 判成已完成。
    assert "frontier_attempts_exhausted_recoverable" in added
    exhausted = patch[patch.index("attempt_summary.allCandidatesSuppressed()") :]
    exhausted = exhausted[: exhausted.index("return;")]
    assert "EXPLORATION_BLOCKED" in exhausted
    assert "EXPLORATION_COMPLETE" not in exhausted
    assert '"no_frontiers"' not in exhausted

    assert "successful_frontier_cooldown" in removed
    assert "frontier_blacklist_" in removed
    assert "frontier_goal_cooldown.h" in patch
    assert "successful_frontier_cooldown" not in added
    assert "frontier_blacklist_" not in added
    assert "/home/ubuntu/" not in patch


def test_frontier_approach_reserves_configured_robot_clearance():
    patch = PATCHES[4].read_text(encoding="utf-8")

    assert "frontier_approach_clearance" in patch
    assert "frontier_approach_max_distance" in patch
    assert "approachCellHasClearance" in patch
    assert "bool has_approach{false}" in patch
    assert "candidate_seen" in patch
    assert "std::hypot(edge_dx, edge_dy)" in patch
    assert "map_[neighbour] != FREE_SPACE" in patch
    assert "navigationGoalKeepsRobotRadiusClearanceFromUnknown" in patch
    assert "unsafeFrontierRemainsVisibleForRecovery" in patch
    assert "no_clearance_safe_frontier_approach" in patch
    assert "approach_max_distance must cover clearance" in patch
    # 净空检查必须把 unknown、障碍和地图边界都视为不可占用，不能仅后移固定坐标。
    assert "neighbour_x < 0 || neighbour_y < 0" in patch
    assert "approach_clearance must be finite" in patch
    # 没有安全执行点只能进入 blocked/recovery；删除 frontier 会伪造 no_frontiers。
    assert "\n+  frontier_list.erase" not in patch


def test_frontier_approach_requires_clearance_safe_path_from_robot():
    patch = PATCHES[5].read_text(encoding="utf-8")

    assert "clearanceReachableFrom" in patch
    assert "lastClearanceDomainValid" in patch
    assert "frontier_robot_anchor_max_distance" in patch
    assert "clearance_reachable_flag[index]" in patch
    assert "neverSelectsLocallySafeGoalBehindNarrowPassage" in patch
    assert "retreatsFromTemporarilyUnsafeRobotCell" in patch
    # 失去安全锚点属于证据不足，必须先于 empty/no_frontiers 完成分支处理。
    anchor_guard = patch.index("if (!search_.lastClearanceDomainValid())")
    complete_guard = patch.index("if (frontiers.empty())")
    assert anchor_guard < complete_guard
    assert "available_frontier_count_ = 0" in patch
    assert "EXPLORATION_IN_PROGRESS" in patch[anchor_guard:complete_guard]


def test_active_frontier_goal_keeps_action_ownership_until_explicit_release():
    patch = PATCHES[6].read_text(encoding="utf-8")

    assert "evaluateActiveFrontierGoal" in patch
    assert "ActiveFrontierGoalReleaseReason" in patch
    assert "UNSAFE_TARGET" in patch
    assert "BLACKLISTED_IDENTITY" in patch
    assert "PROGRESS_TIMEOUT" in patch
    assert "isApproachSafe" in patch
    assert "decision.shouldHold()" in patch
    assert "async_cancel_goal(navigation_goal_handle_)" in patch
    assert "subEpsilonJitterDoesNotRefreshProgress" in patch
    assert "dispatchedApproachIsRecheckedAgainstLatestMap" in patch
    # frontier 每次重排并不代表 Nav2 Action 应被抢占；保持分支必须发生在
    # 新 frontier 的选择之前，只有显式安全/黑名单/超时边界才释放所有权。
    hold_guard = patch.index("if (decision.shouldHold())")
    selection = patch.index("const auto frontier = std::find_if")
    assert hold_guard < selection


def test_frontier_goal_lifecycle_drains_async_callbacks_and_preserves_ledger():
    patch = PATCHES[7].read_text(encoding="utf-8")

    assert "class FrontierGoalLifecycle" in patch
    assert "FrontierGoalState::REQUESTING" in patch
    assert "FrontierGoalState::CANCELING" in patch
    assert "cancel_on_accept_" in patch
    assert "accepted_goal_ids_.insert" in patch
    assert "terminal_goal_ids_.insert" in patch
    assert "ledgerInvariantHolds" in patch
    assert "acceptedGoalOwnsUntilExactlyOneTerminal" in patch
    assert "stopBeforeResponseCountsThenCancelsLateAccepted" in patch
    assert "cancelActiveWaitsForTerminalBeforeHandoff" in patch
    assert "canceledRequestCanDrainThroughRejection" in patch
    assert "staleRejectionCannotSettleCurrentRequest" in patch
    # late accepted 必须先进入 accepted ledger，随后才取消并等待 terminal。
    ledger_insert = patch.index("accepted_goal_ids_.insert")
    cancel_effect = patch.index(
        "FrontierGoalResponseEffects{true, false, false, true}", ledger_insert
    )
    assert ledger_insert < cancel_effect
    # patch 必须可移植，不能泄露生成时的绝对工作树路径。
    assert "/home/ubuntu/" not in patch


def test_frontier_timeout_precedes_tf_and_search_and_rejection_is_explicit():
    patch = PATCHES[7].read_text(encoding="utf-8")

    steady_now = patch.index(
        "const auto planning_now = std::chrono::steady_clock::now()"
    )
    timeout_guard = patch.index("if (goal_lifecycle_.isActiveOwner()")
    tf_guard = patch.index("if (!costmap_client_.getRobotPose(pose))")
    search_call = patch.index("search_.searchFrom(pose.position)")
    assert steady_now < timeout_guard < tf_guard < search_call
    assert "bool getRobotPose(geometry_msgs::msg::Pose& pose) const" in patch
    assert "return false;" in patch

    rejection_start = patch.index("if (!goal_handle) {")
    rejection_end = patch.index(
        "} else if (effects.owns_execution)", rejection_start
    )
    rejection_branch = patch[rejection_start:rejection_end]
    assert "action_server_rejecting" in rejection_branch


def test_frontier_cost_uses_consistent_physical_units():
    patch = PATCHES[8].read_text(encoding="utf-8")
    added = "\n".join(
        line[1:]
        for line in patch.splitlines()
        if line.startswith("+") and not line.startswith("+++")
    )

    # min_distance 在 buildNewFrontier() 中已经由 mapToWorld() 得到米制值；
    # 这里只允许 frontier cell 数乘 resolution，避免分辨率改变目标排序。
    assert "potential_scale_ * frontier.min_distance)" in added
    assert "potential_scale_ * frontier.min_distance *" not in added
    assert "gain_scale_ * frontier.size * costmap_->getResolution()" in patch
    assert "keepsPhysicalScoreAcrossMapResolutions" in patch
    assert "doesNotLetCellCountEraseRealTravelCost" in patch
    assert "/home/ubuntu/" not in patch


def test_approach_reached_tolerance_avoids_false_progress_timeout():
    patch = PATCHES[10].read_text(encoding="utf-8")
    added = "\n".join(
        line[1:]
        for line in patch.splitlines()
        if line.startswith("+") and not line.startswith("+++")
    )

    assert "frontier_approach_reached_tolerance: 0.30" in added
    assert '"frontier_approach_reached_tolerance", 0.30' in added
    assert "std::isfinite(frontier_approach_reached_tolerance_)" in added
    assert "frontier_approach_reached_tolerance_ < 0.0" in added
    assert "must be finite and nonnegative" in added
    assert "APPROACH_REACHED" in added
    assert "isFrontierApproachReached" in added
    assert "std::hypot" in added
    assert "recordReached" in added

    make_plan = patch.index("const auto planning_now")
    active_reached = patch.index("isFrontierApproachReached(", make_plan)
    search = patch.index("search_.searchFrom", active_reached)
    assert active_reached < search
    candidate_reached = patch.index(
        "for (const auto& frontier : approachable_frontiers)", search
    )
    summary = patch.index("summarizeCurrent", candidate_reached)
    assert candidate_reached < summary

    # 进入容差只记 epoch visited 并请求取消，不能绕过 Action result 伪造成功。
    assert "recordReached(active_frontier_goal_)" in added
    assert "ActiveFrontierGoalReleaseReason::APPROACH_REACHED, false" in added
    assert "++succeeded_goal_count_" not in added
    assert "approachReachedPrecedesProgressTimeout" in added
    assert "reachedApproachCancelsWithoutForgingSuccess" in added
    assert "acceptedCount(), 1u" in added
    assert "terminalCount(), 0u" in added
    assert "activeAcceptedCount(), 1u" in added
    assert "/home/ubuntu/" not in patch


def test_frontier_execution_clearance_rejects_critical_pockets_without_overcutting():
    patch = PATCHES[11].read_text(encoding="utf-8")
    added = "\n".join(
        line[1:]
        for line in patch.splitlines()
        if line.startswith("+") and not line.startswith("+++")
    )
    config = yaml.safe_load(FRONTIER_CONFIG.read_text(encoding="utf-8"))
    mission = yaml.safe_load(UNKNOWN_WORLD_MISSION.read_text(encoding="utf-8"))

    clearance = config["/**"]["ros__parameters"][
        "frontier_approach_clearance"
    ]
    goal_clearance = mission["navigation"]["goal_clearance_m"]
    assert clearance == 0.33
    assert goal_clearance == 0.40
    # frontier clearance 会侵蚀整条可达连通域；导航 goal clearance 只约束终点，
    # 两个参数语义不同，强行提高到同一数值会切断本来可执行的通道。
    assert clearance < goal_clearance

    assert "frontier_approach_clearance: 0.33" in added
    assert '"frontier_approach_clearance", 0.33' in added
    assert "quarterMetreClearanceAdmitsCriticalBlindPocket" in added
    assert "executionClearanceRejectsCriticalBlindPocket" in added
    assert "executionClearanceKeepsOneMetreDoorReachable" in added
    assert "fortyCentimetreClearanceOverDisconnectsUsablePassage" in added
    assert "/home/ubuntu/" not in patch


def test_frontier_progress_recovery_counts_motion_and_waits_for_terminal():
    patch = PATCHES[12].read_text(encoding="utf-8")
    added = "\n".join(
        line[1:]
        for line in patch.splitlines()
        if line.startswith("+") and not line.startswith("+++")
    )
    config = yaml.safe_load(FRONTIER_CONFIG.read_text(encoding="utf-8"))
    params = config["/**"]["ros__parameters"]

    assert params["frontier_progress_motion_threshold"] == 0.15
    assert params["max_consecutive_progress_timeouts"] == 2
    assert "anchor_displacement" in added
    assert "made_motion_progress" in added
    assert "ProgressTimeoutCircuitBreaker" in added
    assert "recordApproachFailed" in added
    assert "frontier_progress_stalled_recoverable" in added
    assert "lateralDetourRefreshesExpiredWatchdog" in added
    assert "lateSuccessOrOtherReleaseBreaksStreak" in added
    # BLOCKED 必须从 result callback 的 terminal 事实产生，不能在 cancel
    # request 发出时就解锁上层恢复。
    terminal_index = patch.index("const bool is_current_goal = effects.was_owner")
    breaker_index = patch.index("progress_timeout_breaker_.observeTerminal")
    blocked_index = patch.index("    enterRecoverableProgressBlock();")
    assert terminal_index < breaker_index < blocked_index
    assert "/home/ubuntu/" not in patch


def test_observable_approach_requires_safety_and_escape_breaks_timeout_streak():
    patch = PATCHES[13].read_text(encoding="utf-8")
    added = "\n".join(
        line[1:]
        for line in patch.splitlines()
        if line.startswith("+") and not line.startswith("+++")
    )

    assert "made_meaningful_escape" in added
    assert "goal_max_displacement_m_" in added
    assert "2.0 * frontier_progress_motion_threshold_" in added
    assert "realEscapeBetweenTimeoutsBreaksStreak" in added
    assert "unsafeNearApproachIsNotRecordedAsReached" in added
    assert "isApproachSafe(active_frontier_goal_.approach)" in added
    assert "inside.x = 0.399" in added
    assert "outside.x = 0.401" in added

    # 安全事实必须先于 reached；靠近一个刚变成 unknown/障碍的目标不能写 visited。
    unsafe_index = patch.index("if (!target_safe)")
    reached_index = patch.index("isFrontierApproachReached(", unsafe_index)
    assert unsafe_index < reached_index
    assert "/home/ubuntu/" not in patch


def test_setup_applies_patch_idempotently_without_destructive_git_commands():
    setup = SETUP.read_text(encoding="utf-8")

    assert '"$PATCH_DIR/0003-reachable-frontier-approach.patch"' in setup
    assert '"$PATCH_DIR/0004-successful-frontier-cooldown.patch"' in setup
    assert '"$PATCH_DIR/0005-frontier-approach-clearance.patch"' in setup
    assert '"$PATCH_DIR/0006-clearance-reachable-frontier-path.patch"' in setup
    assert '"$PATCH_DIR/0007-active-frontier-goal-ownership.patch"' in setup
    assert '"$PATCH_DIR/0008-goal-lifecycle-state-machine.patch"' in setup
    assert '"$PATCH_DIR/0009-frontier-cost-metric-units.patch"' in setup
    assert '"$PATCH_DIR/0010-frontier-attempt-memory.patch"' in setup
    assert '"$PATCH_DIR/0011-approach-reached-tolerance.patch"' in setup
    assert '"$PATCH_DIR/0012-execution-clearance-contract.patch"' in setup
    assert '"$PATCH_DIR/0013-frontier-progress-recovery-contract.patch"' in setup
    assert '"$PATCH_DIR/0014-observable-approach-escape-contract.patch"' in setup
    assert "class FrontierGoalLifecycle" in setup
    assert "class FrontierAttemptMemory" in setup
    assert "summarizeCurrent" in setup
    assert "frontier_attempts_exhausted_recoverable" in setup
    assert "APPROACH_REACHED" in setup
    assert "recordReached" in setup
    assert "frontier_approach_reached_tolerance" in setup
    assert "executionClearanceRejectsCriticalBlindPocket" in setup
    assert "frontier_approach_clearance: 0.33" in setup
    assert "frontier_progress_motion_threshold" in setup
    assert "frontier_progress_stalled_recoverable" in setup
    assert "ProgressTimeoutCircuitBreaker" in setup
    assert "recordApproachFailed" in setup
    assert "made_meaningful_escape" in setup
    assert "unsafeNearApproachIsNotRecordedAsReached" in setup
    assert "keepsPhysicalScoreAcrossMapResolutions" in setup
    assert "installed_prefix" in setup
    assert '"${PATCH_FILES[index - 1]}"' in setup
    assert 'apply --check "$patch_file"' in setup
    assert 'apply "$patch_file"' in setup
    assert "validate_explore_status_schema" in setup
    assert "ros2 interface show explore_lite_msgs/msg/ExploreStatus" in setup
    assert re.search(r"git\s+[^\n]*\breset\b", setup) is None
    assert re.search(r"git\s+[^\n]*\bclean\b", setup) is None


def test_frontier_setup_dry_run_documents_patch_and_schema_steps():
    completed = subprocess.run(
        ["bash", str(SETUP), "--dry-run"],
        cwd=ROOT,
        check=False,
        text=True,
        capture_output=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert PINNED_REVISION in completed.stdout
    assert str(PATCHES[0].parent) in completed.stdout
    assert "build and validate its schema" in completed.stdout
