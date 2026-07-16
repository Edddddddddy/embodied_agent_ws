#!/usr/bin/env bash

# 本文件除 Lifecycle 探针外，也承载所有公共入口共用的轻量 shell 运行时契约。
# 复用既有文件可避免每个入口复制路径推导，同时不改变仓库发布物清单。

embodied_resolve_workspace() {
  if [[ $# -ne 1 || -z "${1:-}" ]]; then
    echo "ERROR: embodied_resolve_workspace 需要调用脚本的 BASH_SOURCE[0]。" >&2
    return 2
  fi

  local caller_source="$1"
  local selected_workspace="${WORKSPACE:-}"
  local caller_dir inferred_workspace
  caller_dir="$(cd -- "$(dirname -- "$caller_source")" 2>/dev/null && pwd -P)" || {
    echo "ERROR: 无法定位调用脚本目录：$caller_source" >&2
    return 2
  }
  case "$caller_dir" in
    */scripts) inferred_workspace="${caller_dir%/scripts}" ;;
    */scripts/*) inferred_workspace="${caller_dir%%/scripts/*}" ;;
    *)
      echo "ERROR: 调用脚本不在仓库 scripts 目录中：$caller_source" >&2
      return 2
      ;;
  esac
  inferred_workspace="$(cd -- "$inferred_workspace" 2>/dev/null && pwd -P)" || return 2

  if [[ -z "$selected_workspace" ]]; then
    selected_workspace="$inferred_workspace"
  fi

  if [[ ! -d "$selected_workspace" ]]; then
    echo "ERROR: WORKSPACE 目录不存在：$selected_workspace" >&2
    return 2
  fi
  selected_workspace="$(cd -- "$selected_workspace" 2>/dev/null && pwd -P)" || return 2

  # activate 会 export WORKSPACE；若同一终端随后切换到另一个 worktree，旧值仍存在。
  # 默认拒绝这种不一致，避免再次静默加载旧 install。有意跨目录覆盖必须显式确认。
  if [[ "$selected_workspace" != "$inferred_workspace" &&
        "${EMBODIED_ALLOW_WORKSPACE_OVERRIDE:-false}" != "true" ]]; then
    cat >&2 <<EOF
ERROR: WORKSPACE 与当前入口所属仓库不一致：
  WORKSPACE=$selected_workspace
  script=$inferred_workspace
请先执行 unset WORKSPACE；若确需覆盖，设置 EMBODIED_ALLOW_WORKSPACE_OVERRIDE=true。
EOF
    return 2
  fi

  # 多层 acceptance -> smoke -> launch 子进程必须继承同一个工作区；未 export
  # 会让子脚本重新退回主目录，造成源码与 install 静默错配。
  export WORKSPACE="$selected_workspace"
}

embodied_workspace_doctor() {
  local require_frontier="${1:-false}"
  local install_root="$WORKSPACE/install"
  local failed=0
  local package prefix

  echo "Workspace doctor: $WORKSPACE"
  if [[ ! -f "$install_root/setup.bash" ]]; then
    echo "[FAIL] install overlay: $install_root/setup.bash" >&2
    echo "       修复：colcon build --symlink-install" >&2
    failed=1
  else
    echo "[PASS] install overlay: $install_root/setup.bash"
  fi

  # 不能只判断 package 存在；prefix 必须属于当前 worktree 的 install。
  for package in embodied_agent_interfaces embodied_slam_tools; do
    prefix="$(ros2 pkg prefix "$package" 2>/dev/null || true)"
    case "$prefix" in
      "$install_root"/*) echo "[PASS] package prefix $package: $prefix" ;;
      *)
        echo "[FAIL] package prefix $package: ${prefix:-not found}" >&2
        echo "       修复：重新 source 当前 scripts/activate.sh 并构建当前工作区。" >&2
        failed=1
        ;;
    esac
  done

  if python3 -c 'from embodied_agent_interfaces.action import ManageSlamSession; assert hasattr(ManageSlamSession.Goal, "RUN_AUTOMATIC_MISSION")' 2>/dev/null; then
    echo "[PASS] autonomous SLAM Action: RUN_AUTOMATIC_MISSION"
  else
    echo "[FAIL] autonomous SLAM Action: generated interface is stale" >&2
    echo "       修复：colcon build --symlink-install --packages-up-to embodied_slam_tools" >&2
    failed=1
  fi

  if [[ "$require_frontier" == "true" ]]; then
    prefix="$(ros2 pkg prefix explore_lite 2>/dev/null || true)"
    case "$prefix" in
      "$install_root"/*) echo "[PASS] frontier runtime: $prefix" ;;
      *)
        echo "[FAIL] frontier runtime: ${prefix:-explore_lite not found}" >&2
        echo "       修复：bash scripts/setup_frontier_exploration.sh" >&2
        failed=1
        ;;
    esac
  fi

  if [[ $failed -ne 0 ]]; then
    echo "FAIL: workspace overlay is not ready" >&2
    return 2
  fi
  echo "PASS: workspace overlay is ready"
}

activate_lifecycle_node() {
  local node_name="$1"
  python3 "$WORKSPACE/scripts/activate_lifecycle_node.py" "$node_name"
}

wait_for_topic_subscribers() {
  local topic="$1"
  local minimum="${2:-1}"
  local attempts="${3:-80}"
  local count
  for _ in $(seq 1 "$attempts"); do
    count="$(ros2 topic info "$topic" 2>/dev/null | \
      awk '/Subscription count:/ {print $3}' || true)"
    if [[ "$count" =~ ^[0-9]+$ ]] && (( count >= minimum )); then
      return 0
    fi
    sleep 0.1
  done
  echo "Topic $topic did not discover $minimum subscriber(s)" >&2
  return 1
}
