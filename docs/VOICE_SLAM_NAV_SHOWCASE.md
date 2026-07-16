# 真实感语音 SLAM 与语义导航演示

## 1. 演示目标

这条主演示在一个 10 m × 8 m 的公寓/办公室混合场景中完成：

```text
真实麦克风
  → 在线/离线 ASR
  → 中文命令 NLU/LLM
  → typed RobotCommand / ExecuteRobotCommand
  → C++ ActionGuard 与 ActionScheduler
  → 高层自动任务 / Explore Lite frontier 选点
  → Nav2 NavigateToPose（建图阶段自主探索）
  → TurtleBot3 + LaserScan + Odometry
  → SLAM Toolbox 在线 OccupancyGrid
  → map_saver 保存 YAML/PGM
  → AMCL 定位
  → Nav2RobotExecutor + NavigateToPose
  → 去厨房/办公室/会议区或依次巡航
```

场景包含客厅、厨房、办公室、会议区、走廊、墙体、门洞、桌椅、沙发、柜体、植物和充电桩。
全部几何由项目内 primitive SDF 生成，不在演示时下载 Fuel 资源，因此离线可复现，也不会因模型
版本或网络波动导致现场启动失败。

## 2. 为什么用“一份清单生成三类资产”

唯一人工维护源是：

- `src/embodied_simulation/config/showcase_apartment.yaml`

`scripts/generate_showcase_scene.py` 从它确定性生成：

- `worlds/showcase_apartment.sdf.xacro`：Gazebo 视觉与碰撞几何；
- `maps/showcase_apartment.yaml/.pgm`：现场保底用静态地图；
- `maps/showcase_apartment_slam_frame.yaml`：自动门禁用的完整 SLAM 坐标系地图；
- `src/embodied_simulation/config/showcase_places.yaml`：静态地图/Gazebo 世界坐标下的语义地点；
- `src/embodied_simulation/config/showcase_mapping_places.yaml`：以建图起点为原点的语义地点。

这样避免了“修改墙体后忘记改地图”和“地图正确但目标点落在家具里”两类演示故障。仓库测试还会
按 TurtleBot3 膨胀半径验证所有地点处于同一可达自由空间。

两套地点表不是重复配置。项目静态地图的原点是世界左下角，而 SLAM Toolbox 的 `map` 坐标通常
以建图开始时的机器人为原点。生成器对后者执行：

```text
slam_place.x = world_place.x - spawn.x
slam_place.y = world_place.y - spawn.y
```

因此语音建图后重启机器人时发布 `(0, 0, 0)` 初始位姿，`去厨房` 不会整体偏移一个出生点。
脚本还把 Gazebo spawn 与 AMCL initial pose 拆成两组参数：重启时机器人仍在世界坐标
`(-4.15, -3.15)` 生成，而保存地图中的定位初值是 `(0, 0)`；不能把其中一组坐标同时传给两层。

## 3. 单终端会话编排

主演示由 `voice_slam_session_orchestrator` 维护显式状态机：

```text
STARTING_MAPPING → MAPPING → AUTOMATIC_MAPPING → SAVING_MAP → MAP_SAVED
  → SWITCHING_TO_NAVIGATION → STARTING_NAVIGATION → NAVIGATING
  → AUTOMATIC_NAVIGATING → MISSION_COMPLETED
```

它订阅 `/agent/asr_final` 中少量明确的系统意图，并提供两个 typed ROS 2 接口：

- `/slam/session_state`：`SlamSessionState`，transient-local 的当前阶段快照；
- `/slam/manage_session`：`ManageSlamSession` Action，支持自动任务、保存、启动导航、保存并启动、停止。

自动任务先按 `automatic_exploration.bootstrap_route` 自动脱离充电角；`parse_mapping_bootstrap_route()`
只允许 move/turn，`_run_agent_text_action()` 让每一步继续经过 Agent、ActionGuard 和 ROS 2 Action。
到中央门洞后，固定提交的 `m-explore-ros2/Explore Lite` 提取未知—已知边界并将目标发送给 Nav2；
它不直接发布 `/cmd_vel`。探索结束后的语义导航仍通过 Agent 生成 typed `RobotCommand`，经过 C++
ActionGuard、ROS 2 Action 和 `Nav2RobotExecutor`，没有绕过项目控制面。地图保存和进程切换放在
独立 worker 中，ROS Action/订阅回调不会被 `map_saver_cli` 阻塞。停止阶段
先通知父脚本，让其 trap 有序关闭 launch/monitor；超时后才升级进程组信号。保存失败会回到
`MAPPING` 供用户重试，阶段切换失败才进入 `FAILED`。需要准确区分：高层“开始自动任务”由编排器
校验；bootstrap move/turn 经过 ActionGuard，而后续 frontier goal 由 Explore Lite 直接交给
Nav2。frontier 本身不经过机器人动作 ActionGuard，但受
状态机、任务超时、Nav2 costmap/planner/controller 和取消逻辑约束。存图后的语义导航才重新进入
`RobotCommand → ActionGuard → ExecuteRobotCommand` 主链。

自动任务的前后调用关系为：

```text
SessionOrchestratorNode._on_asr_final()
→ parse_session_command()
→ _enqueue() → _worker_loop() → _execute_request()
→ _run_automatic_mission()
→ parse_mapping_bootstrap_route() → _run_agent_text_action(move/turn)
→ StageProcessManager.start("mapping") / start_explorer()
→ _wait_for_frontier_completion()
→ _save_map() → _start_navigation()
→ _run_agent_text_action()
```

各阶段启动都等待 `/system/readiness` 的新一代 ready 消息。Gazebo、Agent、AMCL/Nav2 并行冷启动
会跨越数秒，组件健康又是 transient-local 状态事件而不是高频心跳，因此语音 Nav2 launch 使用
30 秒 stale 窗口覆盖冷启动；普通执行 launch 仍保持 3 秒默认值。

## 4. 首次准备与主演示

```bash
cd ~/embodied_agent_ws
# 新 clone 先执行：bash scripts/bootstrap.sh
source scripts/activate.sh
embodied_workspace_doctor true
python3 scripts/generate_showcase_scene.py --check
bash scripts/acceptance_test.sh slam-nav-showcase-stage
bash scripts/acceptance_test.sh slam-autonomous-mission-stage
bash scripts/acceptance_test.sh wsl-microphone-preflight
```

单终端启动：

```bash
HEADLESS=false USE_RVIZ=true \
  bash scripts/acceptance_test.sh voice-slam-workplace-demo offline
```

该入口与 `continuous-offline` 使用同一套真实麦克风策略：默认自动加载
`logs/voice_calibration.env`；WSLg Pulse 可用时由 Pulse bridge 从 `@DEFAULT_SOURCE@` 采集，并关闭
重复的 PortAudio capture。校准文件不存在时先运行：

```bash
VOICE_CALIBRATION_COLLECT=true \
  bash scripts/acceptance_test.sh voice-calibration-report
```

不带 `VOICE_CALIBRATION_COLLECT=true` 时使用合成 low-gain 样本，仅用于自动回归。

显式环境变量优先于校准文件，现场排障可设置 `CONTINUOUS_PRINT_CONFIG=true` 核对 profile、VAD、
endpoint/commit delay、Pulse bridge 与实际 capture 状态。

启动完成后说一条高层任务即可：

```text
小智，开始自动巡检建图
```

离线小模型偶尔会把这条领域长句的 final 精确截断为“开始自动”。系统只对白名单中的这个精确
短句恢复自动任务，并可用最新 partial 补回“巡检建图”；“开始”、单独“自动”或完整识别出的其他
意图不会触发任务。若另一条长句也被 ASR 截成完全相同的 final，文本层无法区分，现场需结合
partial 日志确认。若声学链仍不稳定，保持 Terminal 1 运行，在 Terminal 2 使用 typed 备用入口：

```bash
bash scripts/voice_slam_nav_showcase.sh trigger-auto
```

该命令向 `/slam/manage_session` 发送 `RUN_AUTOMATIC_MISSION`，等待任务结果并输出进度。它只绕过
麦克风/ASR 触发层；frontier 探索、地图保存、AMCL/Nav2 切换以及单点/多点导航与语音触发路径
完全相同，因此适合证明机器人闭环，不属于真人 ASR 证据。

机器人会持续选择 frontier 并通过 Nav2 规划、局部控制和代价地图避障。无可达 frontier 后，系统
自动保存地图、重启到 AMCL 定位模式，并顺序完成入口、厨房和办公室任务。建图模式将 Nav2
`xy_goal_tolerance` 从官方默认 0.25 m 收紧为 0.08 m，避免近距离 frontier 被立即判定到达后重复
投递；该参数文件只在建图阶段生成，正常导航阶段恢复官方默认配置。

编排器生成 `logs/showcase/voice_built_map.yaml/.pgm`，关闭 mapping，并以保存地图坐标系的
`(0, 0, 0)` 初值启动 AMCL/Nav2。任务过程中随时可说：

```text
停下
急停
取消自动任务
取消导航
```

AMCL 提供 `map→odom`，Nav2 完成全局规划、局部控制、障碍层检查和 Action 反馈。
`Nav2RobotExecutor` 将语义地点解析为 `NavigateToPose`，巡航失败或取消后必须停车。

## 5. 自动门禁和 PASS 标准

```bash
# 场景、坐标对齐、NLU 和脚本契约
bash scripts/acceptance_test.sh slam-nav-showcase-stage

# dry-run 进程 Adapter：ASR 系统意图、typed Action 与状态机顺序
bash scripts/acceptance_test.sh slam-session-orchestrator-stage
bash scripts/acceptance_test.sh slam-autonomous-mission-stage

# 真正的 frontier 自动探索、存图、定位切换和语义巡检
bash scripts/acceptance_test.sh slam-autonomous-mission

# 固定路线重型回归：用于单独定位底盘控制或地图覆盖故障
bash scripts/acceptance_test.sh slam-session-orchestrator

# 原有分阶段回归
bash scripts/acceptance_test.sh slam-nav-showcase-mapping
bash scripts/acceptance_test.sh slam-nav-showcase
```

完整人工演示的 PASS 条件：

1. 一条“开始自动巡检建图”进入 `AUTOMATIC_MAPPING`，frontier 目标使 `/odom` 改变；
2. 保存得到可加载的 YAML/PGM；
3. 状态机按顺序进入 `NAVIGATING`，且导航重启后出现 `map→odom`；
4. 至少两个语义目标产生规划并成功到达；
5. 结束、失败和取消后 `/cmd_vel` 均归零。

自动任务报告必须进入 `MISSION_COMPLETED`、保存本次会话的新地图、至少包含 6,000 个已知栅格和
150 个占用栅格、建图轨迹不少于 10 m、入口导航和厨房/办公室巡检成功，并验证动态重规划和最终停车。
唯一完整门禁是 `bash scripts/acceptance_test.sh slam-nav-e2e`，报告路径是
`logs/acceptance/slam_nav/<session-id>/slam_nav_e2e_report.json`。自动门禁用 typed Action 触发以保持回归可重复；
`auto offline` 人工验收才是麦克风证据。两者复用相同编排器、Explore Lite、Nav2 和阶段安全检查；
建图 bootstrap 原语与存图后的语义导航都复用
`RobotCommand → ActionGuard → ExecuteRobotCommand` 控制链，不直接写 `/cmd_vel`。

完整重型门禁的关键结果应同时满足：入口 `NavigateToPose` 成功；厨房/办公室
`FollowWaypoints` 的 ROS Action 状态成功、`error_code=0` 且 `missed_waypoints=0`；最后一帧
`/cmd_vel` 的线速度和角速度均为 0。只看到地图文件或机器人移动不能算整条任务通过。

Agent 对 `move/turn` 保持 12 秒快速故障超时，对 `navigate_to/follow_waypoints` 单独使用 330 秒长任务
超时；底层 Nav2 executor 仍有自己的 300 秒 Action 超时。这样 Nav2 可进行规划与恢复，又不会让
普通短动作故障长期占住语音队列。

## 6. 分阶段诊断与事实边界

自动切换异常时，可用原有三阶段命令定位问题：

```bash
bash scripts/voice_slam_nav_showcase.sh mapping offline
bash scripts/voice_slam_nav_showcase.sh save       # mapping 运行时由第二终端执行
bash scripts/voice_slam_nav_showcase.sh navigation offline
```

`navigation` 必须加载本次 `save` 生成的地图。项目不提供静态地图保底入口，避免把预生成地图导航
误报为 SLAM→定位→规划闭环完成。
`ManageSlamSession` 的取消会在等待和保存/切换边界生效；正在运行的 `map_saver_cli` 是有界 35 秒
子进程，当前不做进程内部抢占，不能表述为任意时刻硬实时取消。

本演示证明的是 Gazebo 仿真内的语音—SLAM—Nav2 闭环，不等于真实传感器标定、轮滑、玻璃反射、
动态人群和长期定位稳定性已经解决。真实环境漂移与回环质量仍应使用公开或自采 rosbag 的
ATE/RPE、回环 precision/recall 证据评估。
