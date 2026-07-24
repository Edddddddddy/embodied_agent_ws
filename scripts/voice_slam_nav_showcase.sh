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
MAPPING_PLACES="$WORKSPACE/src/embodied_simulation/config/showcase_mapping_places.yaml"
MISSION_SPEC="$WORKSPACE/src/embodied_simulation/config/showcase_workplace_mission.yaml"
SESSION_DIR="${SHOWCASE_SESSION_DIR:-$WORKSPACE/logs/showcase}"
SAVED_MAP_PREFIX="${SHOWCASE_MAP_PREFIX:-$SESSION_DIR/voice_built_map}"
FRONTIER_NAV2_PARAMS="$SESSION_DIR/frontier_nav2_params.yaml"
FRONTIER_SLAM_PARAMS="$WORKSPACE/src/embodied_simulation/config/frontier_slam_toolbox.yaml"
DYNAMIC_NAV2_PARAMS="$SESSION_DIR/showcase_dynamic_nav2_params.yaml"
SHOWCASE_DYNAMIC_OBSTACLE_ENABLED="${SHOWCASE_DYNAMIC_OBSTACLE_ENABLED:-true}"
SHOWCASE_PERSISTENT_SESSION="${SHOWCASE_PERSISTENT_SESSION:-false}"
SHOWCASE_PRINT_CONFIG="${SHOWCASE_PRINT_CONFIG:-false}"
# 持久 stage 切换必须经过 HOLD/ACK/RESUME；opt-in 后默认同时开启控制权，
# 避免用户还要记住第二个隐藏开关。legacy strict 仍保持 false。
CONTROL_AUTHORITY_ENABLED="${CONTROL_AUTHORITY_ENABLED:-$SHOWCASE_PERSISTENT_SESSION}"
# 只有 auto 会话根可以创建 manager，并把 false 透传给阶段子进程。手工
# mapping/navigation 若开启控制权，必须复用已经运行的外部会话 manager；
# 阶段 launch 不具备聚合旧任务 terminal 证据的资格。
CONTROL_AUTHORITY_MANAGER_ENABLED="${CONTROL_AUTHORITY_MANAGER_ENABLED:-$CONTROL_AUTHORITY_ENABLED}"
SESSION_CONTROL_AUTHORITY_MANAGER_ENABLED="$CONTROL_AUTHORITY_MANAGER_ENABLED"
AUTHORITY_STATE_HEARTBEAT_MS="${AUTHORITY_STATE_HEARTBEAT_MS:-200}"
SYSTEM_READINESS_STALE_TIMEOUT_S="${SYSTEM_READINESS_STALE_TIMEOUT_S:-30.0}"
export CONTROL_AUTHORITY_ENABLED CONTROL_AUTHORITY_MANAGER_ENABLED
export AUTHORITY_STATE_HEARTBEAT_MS SYSTEM_READINESS_STALE_TIMEOUT_S
AUTHORITY_MANAGER_PID=""
SLAM_MISSION_PROFILE="${SLAM_MISSION_PROFILE:-known_world}"
if [[ "$SLAM_MISSION_PROFILE" != "known_world" && \
      "$SLAM_MISSION_PROFILE" != "unknown_world" ]]; then
  echo "FAIL: unsupported SLAM_MISSION_PROFILE=$SLAM_MISSION_PROFILE" >&2
  exit 2
fi
# 在线 frontier 受栅格离散和定位抖动影响更大；只给 unknown-world 较宽默认值，
# known-world 继续用 0.08 m 防止近目标未移动即成功。显式环境变量仍可用于现场诊断。
if [[ "$SLAM_MISSION_PROFILE" == "unknown_world" ]]; then
  FRONTIER_XY_GOAL_TOLERANCE="${FRONTIER_XY_GOAL_TOLERANCE:-0.20}"
else
  FRONTIER_XY_GOAL_TOLERANCE="${FRONTIER_XY_GOAL_TOLERANCE:-0.08}"
fi
export FRONTIER_XY_GOAL_TOLERANCE
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export GZ_PARTITION="${GZ_PARTITION:-embodied_agent_${ROS_DOMAIN_ID}}"
export IGN_PARTITION="${IGN_PARTITION:-$GZ_PARTITION}"

if [[ "$SHOWCASE_DYNAMIC_OBSTACLE_ENABLED" == "true" ]]; then
  SESSION_NAV2_PARAMS="$DYNAMIC_NAV2_PARAMS"
elif [[ "$SHOWCASE_DYNAMIC_OBSTACLE_ENABLED" == "false" ]]; then
  SESSION_NAV2_PARAMS="$FRONTIER_NAV2_PARAMS"
else
  echo "FAIL: SHOWCASE_DYNAMIC_OBSTACLE_ENABLED must be true or false." >&2
  exit 2
fi
if [[ "$SHOWCASE_PERSISTENT_SESSION" != "true" && \
      "$SHOWCASE_PERSISTENT_SESSION" != "false" ]]; then
  echo "FAIL: SHOWCASE_PERSISTENT_SESSION must be true or false." >&2
  exit 2
fi
if [[ "$SHOWCASE_PERSISTENT_SESSION" == "true" && \
      "$CONTROL_AUTHORITY_ENABLED" != "true" ]]; then
  echo "FAIL: persistent session requires CONTROL_AUTHORITY_ENABLED=true." >&2
  echo "      Stage switching must use the typed HOLD/ACK/RESUME transaction." >&2
  exit 2
fi

usage() {
  cat <<'EOF'
真实感语音 SLAM / Nav2 演示

Terminal 1：
  bash scripts/voice_slam_nav_showcase.sh auto offline   # 推荐：办公巡检完整任务
  bash scripts/voice_slam_nav_showcase.sh base offline   # 持久会话内部入口
  bash scripts/voice_slam_nav_showcase.sh mapping offline
  bash scripts/voice_slam_nav_showcase.sh navigation offline

Terminal 2（mapping 仍运行时）：
  bash scripts/voice_slam_nav_showcase.sh save

若长句 ASR 被截断，可在 Terminal 2 使用 typed 备用入口：
  bash scripts/voice_slam_nav_showcase.sh trigger-auto

其他：
  bash scripts/voice_slam_nav_showcase.sh audit

推荐演示顺序：
  1. 推荐 auto：说“小智，开始自动巡检建图”。
  2. 系统自主探索 frontier，结束后自动存图并启动 AMCL/Nav2。
  3. 系统自动执行“去入口”“依次去厨房、办公室”。

mapping/save/navigation 仍保留为手工故障回退。
EOF
}

activate() {
  # PRINT_CONFIG 用于 CI 验证 shell→launch 参数契约，不启动 ROS，也不要求
  # feature worktree 已经生成 install 层。
  [[ "$SHOWCASE_PRINT_CONFIG" != "true" ]] || return 0
  # shellcheck source=activate.sh
  source "$WORKSPACE/scripts/activate.sh"
}

prepare_persistent_session_params() {
  # planner/controller 常驻 base，因此 frontier 与动态障碍覆盖必须在 base
  # 启动前合成一次；三个进程只读取同一个 session 参数文件，禁止阶段漂移。
  export NAV2_PARAMS_FILE="$SESSION_NAV2_PARAMS"
  if [[ "$SHOWCASE_PRINT_CONFIG" == "true" ]]; then
    return 0
  fi
  if [[ "${SHOWCASE_SESSION_PARAMS_PREPARED:-false}" == "true" ]]; then
    if [[ ! -s "$SESSION_NAV2_PARAMS" ]]; then
      echo "FAIL: prepared session params are missing: $SESSION_NAV2_PARAMS" >&2
      return 2
    fi
    return 0
  fi

  # 默认 logs/showcase 会跨多次演示复用目录，因此“文件存在”不能证明它
  # 对应当前源码与配置。每个 runtime 在启动任何 ROS 进程前原子重建一次。
  mkdir -p "$SESSION_DIR"
  local temporary_dir
  temporary_dir="$(mktemp -d "$SESSION_DIR/.nav2-params.XXXXXX")"
  local temporary_frontier="$temporary_dir/frontier_nav2_params.yaml"
  local temporary_dynamic="$temporary_dir/showcase_dynamic_nav2_params.yaml"
  if ! python3 "$WORKSPACE/scripts/prepare_frontier_nav2_params.py" \
    --slam-params-source "$FRONTIER_SLAM_PARAMS" \
    --output "$temporary_frontier" \
    --xy-goal-tolerance "$FRONTIER_XY_GOAL_TOLERANCE"; then
    rm -rf -- "$temporary_dir"
    return 1
  fi
  if [[ "$SHOWCASE_DYNAMIC_OBSTACLE_ENABLED" == "true" ]]; then
    if ! python3 "$WORKSPACE/scripts/build_slam_nav2_params.py" \
      --base "$temporary_frontier" \
      --override "$WORKSPACE/src/embodied_navigation/config/navigation_overrides.yaml" \
      --output "$temporary_dynamic"; then
      rm -rf -- "$temporary_dir"
      return 1
    fi
    mv -f -- "$temporary_dynamic" "$DYNAMIC_NAV2_PARAMS"
  fi
  mv -f -- "$temporary_frontier" "$FRONTIER_NAV2_PARAMS"
  rmdir -- "$temporary_dir"
  export SHOWCASE_SESSION_PARAMS_PREPARED=true
}

run_or_print_ros2_launch() {
  if [[ "$SHOWCASE_PRINT_CONFIG" == "true" ]]; then
    printf 'ros2 launch'
    printf ' %q' "$@"
    printf '\n'
    return 0
  fi
  exec ros2 launch "$@"
}

cleanup_session_authority_manager() {
  local pid="${AUTHORITY_MANAGER_PID:-}"
  [[ -n "$pid" ]] || return 0
  AUTHORITY_MANAGER_PID=""
  # manager 使用独立进程组，确保 ros2 run 包装器及真正 C++ 子进程一起回收。
  kill -TERM -- "-$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}

authority_service_is_ready() {
  ros2 service list 2>/dev/null | grep -Fxq "/control/set_authority"
}

wait_for_authority_service() {
  local attempt
  for attempt in $(seq 1 30); do
    if authority_service_is_ready; then
      return 0
    fi
    if [[ -n "$AUTHORITY_MANAGER_PID" ]] && \
      ! kill -0 "$AUTHORITY_MANAGER_PID" 2>/dev/null; then
      local exit_code=0
      wait "$AUTHORITY_MANAGER_PID" || exit_code=$?
      AUTHORITY_MANAGER_PID=""
      echo "FAIL: session control_authority exited during startup (exit=$exit_code)." >&2
      return 1
    fi
    sleep 0.2
  done
  echo "FAIL: /control/set_authority was not ready within 6s." >&2
  return 1
}

start_session_authority_manager() {
  # auto 的 child mapping/navigation 无论如何都不得再次启动 manager。
  # false 表示调用者已在外部提供 manager；true 表示当前会话根负责其生命周期。
  export CONTROL_AUTHORITY_MANAGER_ENABLED=false
  if [[ "$CONTROL_AUTHORITY_ENABLED" != "true" ]]; then
    if [[ "$SESSION_CONTROL_AUTHORITY_MANAGER_ENABLED" == "true" ]]; then
      echo "FAIL: manager cannot be enabled while control authority is disabled." >&2
      return 2
    fi
    return 0
  fi

  if [[ "$SESSION_CONTROL_AUTHORITY_MANAGER_ENABLED" == "true" ]]; then
    # 只有会话根能把 bootstrap ACK 设为 true，而且必须先证明整套运动栈冷启动。
    # 若存在旧 Gazebo/Nav2/SLAM 进程，本脚本直接失败并要求用户显式清理，
    # 不能在旧 goal 可能存活时伪造“初始静默”。
    bash "$WORKSPACE/scripts/cleanup_simulation_processes.sh"
    if authority_service_is_ready; then
      echo "FAIL: /control/set_authority already exists; refusing a second manager." >&2
      echo "      Set CONTROL_AUTHORITY_MANAGER_ENABLED=false only when that manager is intentional." >&2
      return 2
    fi
    # 使用 wall/system time 让 manager 在 mapping -> navigation 的 Gazebo /clock
    # 间隙仍能持续发布租约心跳；业务节点只按接收时的 steady clock 判定新鲜度。
    setsid ros2 run embodied_agent_cpp control_authority --ros-args \
      -p use_sim_time:=false \
      -p bootstrap_quiescence_acknowledged:=true \
      -p "state_heartbeat_ms:=$AUTHORITY_STATE_HEARTBEAT_MS" &
    AUTHORITY_MANAGER_PID=$!
    echo "[showcase] session control_authority started pid=$AUTHORITY_MANAGER_PID"
  elif [[ "$SESSION_CONTROL_AUTHORITY_MANAGER_ENABLED" == "false" ]]; then
    echo "FAIL: auto showcase must own its session-level control_authority manager." >&2
    echo "      External-manager handoff is not supported until it has a typed coordinator lease." >&2
    return 2
  else
    echo "FAIL: CONTROL_AUTHORITY_MANAGER_ENABLED must be true or false." >&2
    return 2
  fi

  wait_for_authority_service
  # 冷启动许可只表示“可以恢复”，不会自动从 HOLD 进入 AUTONOMY。通过
  # typed service 完成显式 RESUME，并等待同 epoch 状态可见后再启动编排器。
  ros2 run embodied_slam_tools control_authority_bootstrap
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
  export NAV2_ENABLE_DYNAMIC_OBSTACLE_LAYER="$SHOWCASE_DYNAMIC_OBSTACLE_ENABLED"
  export NAV2_INITIAL_X="${NAV2_INITIAL_X:-$initial_x}"
  export NAV2_INITIAL_Y="${NAV2_INITIAL_Y:-$initial_y}"
  export NAV2_INITIAL_YAW="${NAV2_INITIAL_YAW:-$initial_yaw}"
  # 三阶段都在相同 Gazebo 世界位置生成机器人；只有 AMCL 的 map 初始位姿
  # 会在“项目静态地图”和“以建图起点为原点的保存地图”之间切换。
  export NAV2_SPAWN_X="${NAV2_SPAWN_X:--4.15}"
  export NAV2_SPAWN_Y="${NAV2_SPAWN_Y:--3.15}"
  export NAV2_SPAWN_YAW="${NAV2_SPAWN_YAW:-0.0}"
  export NAV_ACTION_TIMEOUT_S="${NAV_ACTION_TIMEOUT_S:-300.0}"
  if [[ "$SHOWCASE_PRINT_CONFIG" == "true" ]]; then
    export CONTINUOUS_PRINT_CONFIG=true
  fi
  exec bash "$WORKSPACE/scripts/continuous_nav2_voice_control.sh" "$MODE"
}

case "$COMMAND" in
  prepare)
    # StageProcessManager 在并发启动 base/mapping 前同步调用一次。该入口
    # 不启动 ROS/Gazebo，仅生成本次会话不可变的 Nav2 参数快照。
    activate
    prepare_persistent_session_params
    echo "[showcase] session Nav2 params prepared: $SESSION_NAV2_PARAMS"
    ;;
  base)
    # base 是 StageProcessManager 的持久 seam：整场只启动一次麦克风、
    # llama/Pulse/monitor、Agent、Gazebo、机器人与 RViz。阶段脚本不得复制这些资源。
    export SHOWCASE_PERSISTENT_SESSION=true
    # base 是持久部署的内部入口，绝不能因脚本解析参数时还处于 legacy
    # 默认值而关闭速度权限门；manager 仍由外层 auto 会话唯一持有。
    export CONTROL_AUTHORITY_ENABLED=true
    export SHOWCASE_SESSION_DIR="$SESSION_DIR"
    export SHOWCASE_MAP_PREFIX="$SAVED_MAP_PREFIX"
    export NAV2_WORLD="$WORLD"
    export NAV2_SPAWN_X="${NAV2_SPAWN_X:--4.15}"
    export NAV2_SPAWN_Y="${NAV2_SPAWN_Y:--3.15}"
    export NAV2_SPAWN_YAW="${NAV2_SPAWN_YAW:-0.0}"
    if [[ "$SLAM_MISSION_PROFILE" == "known_world" ]]; then
      # Agent 与 base 同寿命，语义地点必须在它启动前固定；阶段切换后再 export
      # 不会影响已经运行的进程。
      export NAV2_PLACES_FILE="$MAPPING_PLACES"
    else
      export NAV2_PLACES_FILE=""
    fi
    if [[ "$SHOWCASE_PRINT_CONFIG" == "true" ]]; then
      export CONTINUOUS_PRINT_CONFIG=true
    fi
    activate
    prepare_persistent_session_params
    echo "[showcase] 持久 base：语音支持 + Agent + Gazebo/机器人/RViz + Nav2 common"
    exec bash "$WORKSPACE/scripts/continuous_nav2_voice_control.sh" "$MODE"
    ;;
  auto)
    activate
    print_mission_plan
    echo "[showcase] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    echo "[showcase] GZ_PARTITION=$GZ_PARTITION"
    echo "[showcase] 单终端自动编排：办公巡检建图 -> 保存 -> AMCL -> 多目标 Nav2"
    orchestrator_args=(
      embodied_slam_tools voice_slam_session_orchestrator --ros-args
      -p "workspace:=$WORKSPACE"
      -p "mode:=$MODE"
      -p "map_prefix:=$SAVED_MAP_PREFIX"
      -p "authority_gate_enabled:=$CONTROL_AUTHORITY_ENABLED"
      -p "persistent_runtime_enabled:=$SHOWCASE_PERSISTENT_SESSION"
      -p "dry_run:=${SHOWCASE_ORCHESTRATOR_DRY_RUN:-false}"
    )
    if [[ "$SHOWCASE_PRINT_CONFIG" == "true" ]]; then
      printf 'ros2 run'
      printf ' %q' "${orchestrator_args[@]}"
      printf '\n'
      exit 0
    fi
    # 在拉起 Gazebo 前先失败，避免用户等到状态机内部才看到 explorer 或旧 Action 报错。
    embodied_workspace_doctor true
    # 整场会话只保留一个 manager；StageProcessManager 启停 mapping/navigation
    # 时继承 CONTROL_AUTHORITY_MANAGER_ENABLED=false，因此 epoch 不会随阶段改变。
    trap cleanup_session_authority_manager EXIT INT TERM
    start_session_authority_manager
    status=0
    ros2 run "${orchestrator_args[@]}" || status=$?
    cleanup_session_authority_manager
    trap - EXIT INT TERM
    exit "$status"
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
    if [[ "$SHOWCASE_PERSISTENT_SESSION" == "true" ]]; then
      # 持久阶段只拥有 SLAM provider 与 Gazebo executor；Pulse、llama、
      # monitor、Agent、Gazebo、机器人和 RViz 均由已运行的 base 持有。
      export NAV2_PARAMS_FILE="$SESSION_NAV2_PARAMS"
      if [[ "$SHOWCASE_PRINT_CONFIG" != "true" && \
            ! -s "$SESSION_NAV2_PARAMS" ]]; then
        echo "FAIL: 持久 base 尚未生成会话参数：$SESSION_NAV2_PARAMS" >&2
        echo "      请由 StageProcessManager 先启动 base，再启动 mapping。" >&2
        exit 2
      fi
      echo "[showcase] persistent mapping: session=$SESSION_DIR map_prefix=$SAVED_MAP_PREFIX"
      echo "[showcase] ROS_DOMAIN_ID=$ROS_DOMAIN_ID GZ_PARTITION=$GZ_PARTITION"
      run_or_print_ros2_launch \
        embodied_simulation persistent_mapping_stage.launch.py \
        "params_file:=$SESSION_NAV2_PARAMS" \
        "readiness_stale_timeout_s:=$SYSTEM_READINESS_STALE_TIMEOUT_S" \
        "action_timeout_s:=${NAV_ACTION_TIMEOUT_S:-300.0}"
      exit 0
    fi
    # 建图阶段一次装配 Nav2、frontier 目标容差和 SLAM 扫描接纳策略；切到
    # navigation 后不传该文件，自动恢复 Nav2 官方定位/导航参数。
    if [[ "$SHOWCASE_PRINT_CONFIG" != "true" ]]; then
      python3 "$WORKSPACE/scripts/prepare_frontier_nav2_params.py" \
        --slam-params-source "$FRONTIER_SLAM_PARAMS" \
        --output "$FRONTIER_NAV2_PARAMS" \
        --xy-goal-tolerance "$FRONTIER_XY_GOAL_TOLERANCE"
    fi
    export NAV2_PARAMS_FILE="$FRONTIER_NAV2_PARAMS"
    if [[ "$SLAM_MISSION_PROFILE" == "unknown_world" ]]; then
      # unknown-world 运行时不能把场景真值或预生成地点表放进进程参数；SLAM
      # 模式下 map 本来就是可选项，留空可从结构上证明策略没有读取答案。
      run_voice_stage true "" "embodied_simulation/GazeboRobotExecutor" \
        "" 0.0 0.0 0.0
    else
      run_voice_stage true "$STATIC_MAP" "embodied_simulation/GazeboRobotExecutor" \
        "$MAPPING_PLACES" 0.0 0.0 0.0
    fi
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
    activate
    if [[ "$SHOWCASE_PERSISTENT_SESSION" == "true" ]]; then
      persistent_navigation_args=(
        embodied_simulation persistent_navigation_stage.launch.py
        "map:=$SAVED_MAP_PREFIX.yaml"
        "readiness_stale_timeout_s:=$SYSTEM_READINESS_STALE_TIMEOUT_S"
        "enable_dynamic_obstacle_layer:=$SHOWCASE_DYNAMIC_OBSTACLE_ENABLED"
        "action_timeout_s:=${NAV_ACTION_TIMEOUT_S:-300.0}"
      )
      if [[ "$SHOWCASE_PRINT_CONFIG" != "true" && \
            ! -s "$SESSION_NAV2_PARAMS" ]]; then
        echo "FAIL: 持久 base 尚未生成会话 Nav2 参数：$SESSION_NAV2_PARAMS" >&2
        echo "      请由 StageProcessManager 先启动 base，再切换 navigation。" >&2
        exit 2
      fi
      persistent_navigation_args+=("params_file:=$SESSION_NAV2_PARAMS")
      echo "[showcase] persistent navigation: session=$SESSION_DIR map_prefix=$SAVED_MAP_PREFIX"
      echo "[showcase] ROS_DOMAIN_ID=$ROS_DOMAIN_ID GZ_PARTITION=$GZ_PARTITION"
      # persistent 会话的 /initialpose 由 SessionOrchestrator 在 AMCL ACTIVE、
      # 订阅匹配之后发布，并等待本代 /amcl_pose。shell 后台 helper 无法观察
      # lifecycle/generation，也无法把提前退出传播给会话，因此禁止在这里竞态启动。
      run_or_print_ros2_launch "${persistent_navigation_args[@]}"
      exit 0
    fi
    if [[ "$SHOWCASE_DYNAMIC_OBSTACLE_ENABLED" == "true" ]]; then
      # 只在导航阶段插入预测层；SLAM 阶段不能让演示障碍污染待保存地图。
      python3 "$WORKSPACE/scripts/build_slam_nav2_params.py" \
        --output "$DYNAMIC_NAV2_PARAMS"
      export NAV2_PARAMS_FILE="$DYNAMIC_NAV2_PARAMS"
    fi
    # 保存的 SLAM 地图以建图起点为 map 原点，因此使用相对起点的地点表，
    # 并把重启后的初始位姿发布为 (0, 0, 0)。
    if [[ "$SLAM_MISSION_PROFILE" == "unknown_world" ]]; then
      run_voice_stage false "$SAVED_MAP_PREFIX.yaml" \
        "embodied_simulation/Nav2RobotExecutor" "" 0.0 0.0 0.0
    else
      run_voice_stage false "$SAVED_MAP_PREFIX.yaml" \
        "embodied_simulation/Nav2RobotExecutor" "$MAPPING_PLACES" 0.0 0.0 0.0
    fi
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
