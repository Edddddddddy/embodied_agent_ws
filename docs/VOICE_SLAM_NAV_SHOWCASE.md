# 真实感语音 SLAM 与语义导航演示

## 1. 演示目标

这条主演示在一个 10 m × 8 m 的公寓/办公室混合场景中完成：

```text
真实麦克风
  → 在线/离线 ASR
  → 中文命令 NLU/LLM
  → typed RobotCommand / ExecuteRobotCommand
  → C++ ActionGuard 与 ActionScheduler
  → GazeboRobotExecutor（建图阶段的速度动作）
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
- `config/showcase_places.yaml`：静态地图/Gazebo 世界坐标下的语义地点；
- `config/showcase_mapping_places.yaml`：以建图起点为原点的语义地点。

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
STARTING_MAPPING → MAPPING → SAVING_MAP → MAP_SAVED
  → SWITCHING_TO_NAVIGATION → STARTING_NAVIGATION → NAVIGATING
```

它订阅 `/agent/asr_final` 中少量明确的系统意图，并提供两个 typed ROS 2 接口：

- `/slam/session_state`：`SlamSessionState`，transient-local 的当前阶段快照；
- `/slam/manage_session`：`ManageSlamSession` Action，支持保存、启动导航、保存并启动、停止。

地图保存和进程切换放在独立 worker 中，ROS Action/订阅回调不会被 `map_saver_cli` 阻塞。停止阶段
先通知父脚本，让其 trap 有序关闭 launch/monitor；超时后才升级进程组信号。保存失败会回到
`MAPPING` 供用户重试，阶段切换失败才进入 `FAILED`。SLAM/导航动作仍走原有
`RobotCommand → ActionGuard → ExecuteRobotCommand`，系统意图没有绕过安全控制面。

各阶段启动都等待 `/system/readiness` 的新一代 ready 消息。Gazebo、Agent、AMCL/Nav2 并行冷启动
会跨越数秒，组件健康又是 transient-local 状态事件而不是高频心跳，因此语音 Nav2 launch 使用
30 秒 stale 窗口覆盖冷启动；普通执行 launch 仍保持 3 秒默认值。

## 4. 首次准备与主演示

```bash
cd ~/embodied_agent_ws
source scripts/activate.sh
python3 scripts/generate_showcase_scene.py --check
bash scripts/acceptance_test.sh slam-nav-showcase-stage
bash scripts/acceptance_test.sh wsl-microphone-preflight
```

单终端启动：

```bash
HEADLESS=false USE_RVIZ=true \
  bash scripts/acceptance_test.sh voice-slam-workplace-demo offline
```

脚本会从 `config/showcase_workplace_mission.yaml` 打印确定性办公巡检任务。每条动作完成后再说下一条：

```text
小智
前进三秒
左转九十度
前进十秒
前进四秒
右转九十度
前进五秒
左转九十度
前进八秒
后退八秒
右转九十度
前进十秒
前进十秒
前进两秒
左转九十度
前进五秒
保存地图并开始导航
```

路线让 3.5 m 激光分别观察客厅、厨房、走廊和办公室；门洞与办公桌附近的动作长度按 TurtleBot3
膨胀半径留出余量，雷达安全层仍可对意外接近家具的动作触发 `front_emergency`。

建图阶段选择 `GazeboRobotExecutor`，让 `move/turn/stop` 直接控制底盘；此时使用
`Nav2RobotExecutor` 会形成“还没有完整地图，却要求 Nav2 先规划探索动作”的循环依赖。说出阶段
命令后，编排器生成 `logs/showcase/voice_built_map.yaml/.pgm`，关闭 mapping，并以保存地图坐标系的
`(0, 0, 0)` 初值启动 AMCL/Nav2。终端出现 `session phase=navigating` 后继续说：

```text
小智，去入口
依次去厨房、办公室
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

# 真实重型闭环：Gazebo 探索、在线 /map、map_saver、AMCL/Nav2 和实际移动
bash scripts/acceptance_test.sh slam-session-orchestrator

# 原有分阶段回归
bash scripts/acceptance_test.sh slam-nav-showcase-mapping
bash scripts/acceptance_test.sh slam-nav-showcase
```

完整人工演示的 PASS 条件：

1. 建图时 `/map` 发布，语音动作使 `/odom` 改变；
2. 保存得到可加载的 YAML/PGM；
3. 状态机按顺序进入 `NAVIGATING`，且导航重启后出现 `map→odom`；
4. 至少两个语义目标产生规划并成功到达；
5. 结束、失败和取消后 `/cmd_vel` 均归零。

重型自动报告还必须包含状态阶段 1～7、15 个成功建图步骤、至少 10 m 建图路径、至少 6,000 个
已知栅格、至少 150 个占用栅格、入口导航和厨房/办公室巡检成功，并验证 `map→base` 定位。
报告路径是 `logs/showcase/orchestrated_runtime/workplace_mission_report.json`。自动门禁用文本 topic
代替真人发声以保持回归可重复；
`auto offline` 人工验收才是麦克风证据。两者复用相同 Agent、typed Action、ActionGuard 和 executor。

Agent 对 `move/turn` 保持 12 秒快速故障超时，对 `navigate_to/follow_waypoints` 单独使用 330 秒长任务
超时；底层 Nav2 executor 仍有自己的 300 秒 Action 超时。这样 Nav2 可进行规划与恢复，又不会让
普通短动作故障长期占住语音队列。

## 6. 手工回退、现场保底与事实边界

自动切换异常时，可用原有三阶段命令定位问题：

```bash
bash scripts/voice_slam_nav_showcase.sh mapping offline
bash scripts/voice_slam_nav_showcase.sh save       # mapping 运行时由第二终端执行
bash scripts/voice_slam_nav_showcase.sh navigation offline
```

现场时间不足或探索覆盖不全时，可使用：

```bash
bash scripts/voice_slam_nav_showcase.sh navigation-static offline
```

它加载与 Gazebo 几何同源生成的确定性地图，不能宣称该地图由本次语音探索建立。
`ManageSlamSession` 的取消会在等待和保存/切换边界生效；正在运行的 `map_saver_cli` 是有界 35 秒
子进程，当前不做进程内部抢占，不能表述为任意时刻硬实时取消。

本演示证明的是 Gazebo 仿真内的语音—SLAM—Nav2 闭环，不等于真实传感器标定、轮滑、玻璃反射、
动态人群和长期定位稳定性已经解决。真实环境漂移与回环质量仍应使用公开或自采 rosbag 的
ATE/RPE、回环 precision/recall 证据评估。
