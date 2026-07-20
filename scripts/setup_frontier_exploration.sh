#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source "$SCRIPT_DIR/lifecycle_utils.sh"
embodied_resolve_workspace "${BASH_SOURCE[0]}"
REPOSITORY="https://github.com/robo-friends/m-explore-ros2.git"
REVISION="326cf8a0b487c34246bb8f3326afbcd69576dc60"
SOURCE_DIR="$WORKSPACE/third_party/m-explore-ros2"
PATCH_DIR="$WORKSPACE/patches/m-explore-ros2"
PATCH_FILES=(
  "$PATCH_DIR/0001-frontier-telemetry-tf.patch"
  "$PATCH_DIR/0002-goal-ledger-cooldown.patch"
  "$PATCH_DIR/0003-reachable-frontier-approach.patch"
  "$PATCH_DIR/0004-successful-frontier-cooldown.patch"
  "$PATCH_DIR/0005-frontier-approach-clearance.patch"
  "$PATCH_DIR/0006-clearance-reachable-frontier-path.patch"
  "$PATCH_DIR/0007-active-frontier-goal-ownership.patch"
  "$PATCH_DIR/0008-goal-lifecycle-state-machine.patch"
  "$PATCH_DIR/0009-frontier-cost-metric-units.patch"
  "$PATCH_DIR/0010-frontier-attempt-memory.patch"
  "$PATCH_DIR/0011-approach-reached-tolerance.patch"
  "$PATCH_DIR/0012-execution-clearance-contract.patch"
  "$PATCH_DIR/0013-frontier-progress-recovery-contract.patch"
  "$PATCH_DIR/0014-observable-approach-escape-contract.patch"
)
DRY_RUN=false
PREPARE_ONLY=false

fail_patch_state() {
  echo "FAIL: m-explore-ros2 patch state is neither clean/applicable nor already applied" >&2
  echo "Inspect without deleting local work: git -C '$SOURCE_DIR' status --short" >&2
  echo "Then reconcile '$PATCH_DIR' manually; this script never resets or cleans the checkout." >&2
  exit 2
}

apply_pinned_patches() {
  local patch_file
  for patch_file in "${PATCH_FILES[@]}"; do
    if [[ ! -f "$patch_file" ]]; then
      echo "FAIL: required patch is missing: $patch_file" >&2
      exit 2
    fi
  done

  local head_revision
  head_revision="$(git -C "$SOURCE_DIR" rev-parse HEAD)"
  if [[ "$head_revision" != "$REVISION" ]]; then
    echo "FAIL: expected m-explore-ros2 revision $REVISION, got $head_revision" >&2
    exit 2
  fi

  # 补丁严格按编号依赖。由最新补丁向前寻找“已安装前缀”，再只追加缺少
  # 的后缀；reverse 仅做无写入探测，整个流程绝不 reset/clean 用户现场。
  local installed_prefix=0
  local index
  for ((index=${#PATCH_FILES[@]}; index >= 1; --index)); do
    if git -C "$SOURCE_DIR" apply --reverse --check \
        "${PATCH_FILES[index - 1]}" >/dev/null 2>&1; then
      installed_prefix="$index"
      break
    fi
  done

  if ((installed_prefix > 0)); then
    for ((index=installed_prefix; index < ${#PATCH_FILES[@]}; ++index)); do
      if ! git -C "$SOURCE_DIR" apply --check "${PATCH_FILES[index]}"; then
        fail_patch_state
      fi
      git -C "$SOURCE_DIR" apply "${PATCH_FILES[index]}"
      echo "frontier patch: upgraded with $(basename "${PATCH_FILES[index]}")"
    done
    if ! grep -q "class FrontierGoalLifecycle" \
        "$SOURCE_DIR/explore/include/explore/frontier_goal_lifecycle.h" \
        || ! grep -q "goal_lifecycle_" \
          "$SOURCE_DIR/explore/include/explore/explore.h" \
        || ! grep -q "bool getRobotPose" \
          "$SOURCE_DIR/explore/include/explore/costmap_client.h" \
        || ! grep -q "geometry_msgs::msg::Point approach" \
          "$SOURCE_DIR/explore/include/explore/frontier_search.h" \
        || ! grep -q "class FrontierAttemptMemory" \
          "$SOURCE_DIR/explore/include/explore/frontier_attempt_memory.h" \
        || ! grep -q "summarizeCurrent" \
          "$SOURCE_DIR/explore/include/explore/frontier_attempt_memory.h" \
        || ! grep -q "frontier_attempt_memory_" \
          "$SOURCE_DIR/explore/include/explore/explore.h" \
        || ! grep -q "frontier_attempts_exhausted_recoverable" \
          "$SOURCE_DIR/explore/src/explore.cpp" \
        || ! grep -q "APPROACH_REACHED" \
          "$SOURCE_DIR/explore/include/explore/active_frontier_goal_policy.h" \
        || ! grep -q "recordReached" \
          "$SOURCE_DIR/explore/include/explore/frontier_attempt_memory.h" \
        || ! grep -q "frontier_approach_reached_tolerance" \
          "$SOURCE_DIR/explore/config/params.yaml" \
        || ! grep -q "reachedApproachCancelsWithoutForgingSuccess" \
          "$SOURCE_DIR/explore/test/test_explore.cpp" \
        || ! grep -q "frontier_approach_clearance: 0.33" \
          "$SOURCE_DIR/explore/config/params.yaml" \
        || ! grep -q "executionClearanceRejectsCriticalBlindPocket" \
          "$SOURCE_DIR/explore/test/test_explore.cpp" \
        || ! grep -q "frontier_progress_motion_threshold" \
          "$SOURCE_DIR/explore/config/params.yaml" \
        || ! grep -q "frontier_progress_stalled_recoverable" \
          "$SOURCE_DIR/explore/src/explore.cpp" \
        || ! grep -q "ProgressTimeoutCircuitBreaker" \
          "$SOURCE_DIR/explore/include/explore/active_frontier_goal_policy.h" \
        || ! grep -q "recordApproachFailed" \
          "$SOURCE_DIR/explore/include/explore/frontier_attempt_memory.h" \
        || ! grep -q "made_meaningful_escape" \
          "$SOURCE_DIR/explore/include/explore/active_frontier_goal_policy.h" \
        || ! grep -q "unsafeNearApproachIsNotRecordedAsReached" \
          "$SOURCE_DIR/explore/test/test_explore.cpp" \
        || [[ -e \
          "$SOURCE_DIR/explore/include/explore/frontier_goal_cooldown.h" ]] \
        || grep -q "successful_frontier_cooldown" \
          "$SOURCE_DIR/explore/config/params.yaml" \
        || grep -q "frontier_blacklist_" \
          "$SOURCE_DIR/explore/include/explore/explore.h" \
        || ! grep -q "approachCellHasClearance" \
          "$SOURCE_DIR/explore/include/explore/frontier_search.h" \
        || ! grep -q "lastClearanceDomainValid" \
          "$SOURCE_DIR/explore/include/explore/frontier_search.h" \
        || ! grep -q "evaluateActiveFrontierGoal" \
          "$SOURCE_DIR/explore/include/explore/active_frontier_goal_policy.h" \
        || ! grep -q "isApproachSafe" \
          "$SOURCE_DIR/explore/include/explore/frontier_search.h" \
        || ! grep -q "potential_scale_ \* frontier.min_distance)" \
          "$SOURCE_DIR/explore/src/frontier_search.cpp" \
        || ! grep -q "keepsPhysicalScoreAcrossMapResolutions" \
          "$SOURCE_DIR/explore/test/test_explore.cpp" \
        || ! grep -q "EXPLORATION_BLOCKED" \
          "$SOURCE_DIR/explore_lite_msgs/msg/ExploreStatus.msg"; then
      fail_patch_state
    fi
    echo "frontier patches: installed prefix ${#PATCH_FILES[@]}/${#PATCH_FILES[@]}"
    return
  fi

  # 第三方目录可能包含用户正在调试的修改。补丁不可逆向识别时，只有完全
  # 干净的 pinned checkout 才允许写入；绝不 reset/clean 掩盖或删除现场。
  if [[ -n "$(git -C "$SOURCE_DIR" status --porcelain)" ]]; then
    fail_patch_state
  fi
  for patch_file in "${PATCH_FILES[@]}"; do
    if ! git -C "$SOURCE_DIR" apply --check "$patch_file"; then
      fail_patch_state
    fi
    git -C "$SOURCE_DIR" apply "$patch_file"
    echo "frontier patch: applied $(basename "$patch_file")"
  done
}

validate_explore_status_schema() {
  local schema
  if ! schema="$(ros2 interface show explore_lite_msgs/msg/ExploreStatus)"; then
    echo "FAIL: ExploreStatus interface cannot be inspected after build" >&2
    exit 2
  fi

  local field
  local expected_fields=(
    detected_frontier_count available_frontier_count
    blacklisted_frontier_count active_goal_count active_goal_id
    accepted_goal_count succeeded_goal_count aborted_goal_count
    canceled_goal_count rejected_goal_count last_goal_terminal completion_reason
  )
  for field in "${expected_fields[@]}"; do
    if ! grep -Eq "^[[:space:]]*(uint32|string)[[:space:]]+$field([[:space:]]|$)" <<<"$schema"; then
      echo "FAIL: ExploreStatus schema is missing field: $field" >&2
      exit 2
    fi
  done
  if ! grep -q "EXPLORATION_BLOCKED=exploration_blocked" <<<"$schema"; then
    echo "FAIL: ExploreStatus schema is missing recoverable blocked state" >&2
    exit 2
  fi
}

case "${1:-}" in
  "") ;;
  --dry-run) DRY_RUN=true ;;
  --prepare-only) PREPARE_ONLY=true ;;
  *)
    echo "Usage: $0 [--dry-run|--prepare-only]" >&2
    exit 2
    ;;
esac

echo "frontier explorer: $REPOSITORY@$REVISION"
echo "source: $SOURCE_DIR"
if [[ "$DRY_RUN" == "true" ]]; then
  echo "DRY RUN: clone/fetch pinned m-explore-ros2, apply $PATCH_DIR in order, then build and validate its schema"
  exit 0
fi

if [[ ! -d "$SOURCE_DIR/.git" ]]; then
  mkdir -p "$(dirname "$SOURCE_DIR")"
  git clone "$REPOSITORY" "$SOURCE_DIR"
elif [[ "$(git -C "$SOURCE_DIR" remote get-url origin)" != "$REPOSITORY" ]]; then
  echo "FAIL: $SOURCE_DIR points to an unexpected Git remote" >&2
  exit 2
fi

git -C "$SOURCE_DIR" fetch --tags origin
if ! git -C "$SOURCE_DIR" checkout --detach "$REVISION"; then
  echo "FAIL: checkout refused to preserve local m-explore-ros2 changes; inspect $SOURCE_DIR" >&2
  exit 2
fi
apply_pinned_patches

# CI 的 patch-replay job 只需要一份可编译的干净源码树；提前返回可避免
# 依赖当前工作区已有 install overlay，也让“准备上游”和“构建测试”职责分离。
if [[ "$PREPARE_ONLY" == "true" ]]; then
  echo "PASS: pinned Explore Lite checkout and patch stack are ready"
  exit 0
fi

# shellcheck source=activate.sh
source "$WORKSPACE/scripts/activate.sh"
cd "$WORKSPACE"
colcon build --symlink-install \
  --base-paths "$SOURCE_DIR" \
  --packages-select explore_lite_msgs explore_lite

# ROS/colcon 生成的 setup 脚本会读取未定义的 COLCON_TRACE；本脚本启用了
# `set -u`，因此只在 source 期间临时关闭 nounset，避免“构建成功却验收失败”。
set +u
source "$WORKSPACE/install/setup.bash"
set -u
ros2 pkg executables explore_lite | grep -q 'explore_lite explore'
validate_explore_status_schema
embodied_workspace_doctor true
echo "PASS: pinned and patched Explore Lite frontier runtime is installed"
