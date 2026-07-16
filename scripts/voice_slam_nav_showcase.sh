#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source "$SCRIPT_DIR/lifecycle_utils.sh"
embodied_resolve_workspace "${BASH_SOURCE[0]}"
COMMAND="${1:-help}"
MODE="${2:-offline}"
SCENE_SPEC="$WORKSPACE/src/embodied_simulation/config/showcase_apartment.yaml"
WORLD="$WORKSPACE/src/embodied_simulation/worlds/showcase_apartment.sdf.xacro"
STATIC_MAP="$WORKSPACE/src/embodied_simulation/maps/showcase_apartment.yaml"
STATIC_PLACES="$WORKSPACE/src/embodied_simulation/config/showcase_places.yaml"
MAPPING_PLACES="$WORKSPACE/src/embodied_simulation/config/showcase_mapping_places.yaml"
MISSION_SPEC="$WORKSPACE/src/embodied_simulation/config/showcase_workplace_mission.yaml"
SESSION_DIR="${SHOWCASE_SESSION_DIR:-$WORKSPACE/logs/showcase}"
SAVED_MAP_PREFIX="${SHOWCASE_MAP_PREFIX:-$SESSION_DIR/voice_built_map}"
FRONTIER_NAV2_PARAMS="$SESSION_DIR/frontier_nav2_params.yaml"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"

usage() {
  cat <<'EOF'
真实感语音 SLAM / Nav2 演示

Terminal 1：
  bash scripts/voice_slam_nav_showcase.sh auto offline   # 推荐：办公巡检完整任务
  bash scripts/voice_slam_nav_showcase.sh mapping offline
  bash scripts/voice_slam_nav_showcase.sh navigation offline

Terminal 2（mapping 仍运行时）：
  bash scripts/voice_slam_nav_showcase.sh save

若长句 ASR 被截断，可在 Terminal 2 使用 typed 备用入口：
  bash scripts/voice_slam_nav_showcase.sh trigger-auto

其他：
  bash scripts/voice_slam_nav_showcase.sh audit
  bash scripts/voice_slam_nav_showcase.sh navigation-static offline

推荐演示顺序：
  1. 推荐 auto：说“小智，开始自动巡检建图”。
  2. 系统自主探索 frontier，结束后自动存图并启动 AMCL/Nav2。
  3. 系统自动执行“去入口”“依次去厨房、办公室”。

mapping/save/navigation 仍保留为手工故障回退。

navigation-static 使用项目自带确定性地图，适合作为现场演示的保底路径。
EOF
}

activate() {
  # shellcheck source=activate.sh
  source "$WORKSPACE/scripts/activate.sh"
}

print_mission_plan() {
  python3 - "$MISSION_SPEC" <<'PY'
import sys
from pathlib import Path

import yaml

mission = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8"))
print(f"[showcase] 任务：{mission['description']}")
print("[showcase] 推荐：只说一条“小智，开始自动巡检建图”")
print("[showcase] 系统将自主探索、保存地图、切换定位并执行语义巡检。")
print("[showcase] 手工故障回退话术（只有自动探索异常时才逐条使用）：")
for index, step in enumerate(mission["mapping_route"], 1):
    print(f"  {index:02d}. {step['text']}  # {step['label']}")
print("  16. 保存地图并开始导航")
print("[showcase] 进入 NAVIGATING 后继续说：")
print(f"  17. {mission['navigation_mission']['navigate_text']}")
print(f"  18. {mission['navigation_mission']['patrol_text']}")
print("  19. 结束建图演示")
PY
}

run_voice_stage() {
  local slam="$1"
  local map="$2"
  local executor_plugin="$3"
  local places="$4"
  local initial_x="$5"
  local initial_y="$6"
  local initial_yaw="$7"
  export HEADLESS="${HEADLESS:-false}"
  export USE_RVIZ="${USE_RVIZ:-true}"
  export NAV2_SLAM="$slam"
  export NAV2_WORLD="$WORLD"
  export NAV2_MAP="$map"
  export NAV2_PLACES_FILE="$places"
  export NAV2_EXECUTOR_PLUGIN="$executor_plugin"
  export NAV2_INITIAL_X="${NAV2_INITIAL_X:-$initial_x}"
  export NAV2_INITIAL_Y="${NAV2_INITIAL_Y:-$initial_y}"
  export NAV2_INITIAL_YAW="${NAV2_INITIAL_YAW:-$initial_yaw}"
  # 三阶段都在相同 Gazebo 世界位置生成机器人；只有 AMCL 的 map 初始位姿
  # 会在“项目静态地图”和“以建图起点为原点的保存地图”之间切换。
  export NAV2_SPAWN_X="${NAV2_SPAWN_X:--4.15}"
  export NAV2_SPAWN_Y="${NAV2_SPAWN_Y:--3.15}"
  export NAV2_SPAWN_YAW="${NAV2_SPAWN_YAW:-0.0}"
  export NAV_ACTION_TIMEOUT_S="${NAV_ACTION_TIMEOUT_S:-300.0}"
  exec bash "$WORKSPACE/scripts/continuous_nav2_voice_control.sh" "$MODE"
}

case "$COMMAND" in
  auto)
    activate
    # 在拉起 Gazebo 前先失败，避免用户等到状态机内部才看到 explorer 或旧 Action 报错。
    embodied_workspace_doctor true
    print_mission_plan
    echo "[showcase] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    echo "[showcase] 单终端自动编排：办公巡检建图 -> 保存 -> AMCL -> 多目标 Nav2"
    exec ros2 run embodied_slam_tools voice_slam_session_orchestrator --ros-args \
      -p "workspace:=$WORKSPACE" \
      -p "mode:=$MODE" \
      -p "map_prefix:=$SAVED_MAP_PREFIX" \
      -p "dry_run:=${SHOWCASE_ORCHESTRATOR_DRY_RUN:-false}"
    ;;
  trigger-auto)
    activate
    echo "[showcase] ROS_DOMAIN_ID=$ROS_DOMAIN_ID；等待 mapping ready 后发送 typed Action。"
    exec python3 "$WORKSPACE/scripts/trigger_automatic_slam_mission.py"
    ;;
  mapping)
    echo "[showcase] 阶段 1/3：真实感室内场景 + SLAM Toolbox 在线建图"
    echo "[showcase] 另开终端运行 save 后，再 Ctrl+C 结束本阶段。"
    activate
    # Nav2 默认 0.25 m 目标容差可能让近处 frontier 在机器人尚未移动时即成功，
    # 造成 Explore Lite 重复投递同一目标。这里只给建图阶段生成 0.08 m 配置；
    # 切到 navigation 后不再传该文件，自动恢复 Nav2 官方容差。
    python3 "$WORKSPACE/scripts/prepare_frontier_nav2_params.py" \
      --output "$FRONTIER_NAV2_PARAMS"
    export NAV2_PARAMS_FILE="$FRONTIER_NAV2_PARAMS"
    run_voice_stage true "$STATIC_MAP" "embodied_simulation/GazeboRobotExecutor" \
      "$MAPPING_PLACES" 0.0 0.0 0.0
    ;;
  save)
    activate
    mkdir -p "$SESSION_DIR"
    echo "[showcase] 阶段 2/3：保存 /map -> $SAVED_MAP_PREFIX.{yaml,pgm}"
    ros2 run nav2_map_server map_saver_cli \
      -f "$SAVED_MAP_PREFIX" \
      --ros-args -p save_map_timeout:=15.0 -p free_thresh_default:=0.25 -p occupied_thresh_default:=0.65
    test -s "$SAVED_MAP_PREFIX.yaml"
    test -s "$SAVED_MAP_PREFIX.pgm"
    echo "PASS: map saved; stop mapping and start navigation"
    ;;
  navigation)
    test -s "$SAVED_MAP_PREFIX.yaml" || {
      echo "FAIL: 未找到 $SAVED_MAP_PREFIX.yaml；请先在 mapping 运行时执行 save。" >&2
      exit 2
    }
    echo "[showcase] 阶段 3/3：加载语音建图结果 + AMCL 定位 + Nav2 规划/避障"
    # 保存的 SLAM 地图以建图起点为 map 原点，因此使用相对起点的地点表，
    # 并把重启后的初始位姿发布为 (0, 0, 0)。
    run_voice_stage false "$SAVED_MAP_PREFIX.yaml" "embodied_simulation/Nav2RobotExecutor" \
      "$MAPPING_PLACES" 0.0 0.0 0.0
    ;;
  navigation-static)
    echo "[showcase] 保底演示：加载项目内确定性地图 + AMCL + Nav2"
    run_voice_stage false "$STATIC_MAP" "embodied_simulation/Nav2RobotExecutor" \
      "$STATIC_PLACES" -4.15 -3.15 0.0
    ;;
  audit)
    python3 "$WORKSPACE/scripts/generate_showcase_scene.py" --spec "$SCENE_SPEC" --check
    ;;
  help|-h|--help)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
