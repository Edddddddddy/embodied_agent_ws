# 未知场地 SLAM 收敛、返航与导航 Goal

> 状态：**已完成 RViz 可视化稳定性验收；session `20260725T120302Z-145519-3ec3df78` 全部 checks 通过**
> 稳定基线：`dev@90e77b5`
> 稳定性修复：`fix/gui-slam-runtime-stability`
> 正式入口：`unknown-world-slam-e2e`
> 真人入口：`voice-unknown-world-slam-e2e {offline|online}`

## 1. 目标

在不知道地图尺寸、房间坐标和预设路线的前提下，完成一条有界且可审计的事务：

```text
实时 scan/odom/TF/map
→ frontier 自主探索与恢复
→ 严格终结，或硬预算后的收益递减评估
→ 优雅暂停 Explorer、排空 Nav2 Action 总账
→ 最终 360° 探测、地图静默、typed STOP
→ 回到首次运动前动态捕获的起点
→ 等待 SLAM/回环尾帧并保存本次地图
→ 切换 map_server + AMCL + Nav2
→ 3 个本次地图运行时采样目标 + 动态障碍重规划
→ 最终新鲜零速度和同 session 报告
```

本 Goal 解决现场观察到的问题：地图已覆盖大部分可达区域，但少量分散在墙角或障碍物后的 frontier
长期无法到达，机器人看起来停住，最终被验收清理流程结束。修复不能依赖真值地图，也不能把“900 秒用完”
直接改写成成功。

## 2. 完成语义

探索有两条互斥的生产侧收口路径。

### 2.1 严格 frontier 终结

优先沿用 provider 的强类型终态：

- `no_frontiers + no_reachable_frontiers`；或
- typed attempts exhaustion 经恢复扫描证明低增益/完成最终确认。

该路径仍要求没有 active goal，且 accepted goal 全部进入 succeeded/aborted/canceled 终态。不能用日志
字符串、`all-blacklisted` 或一次短暂停滞代替这些条件。

### 2.2 未知总面积下的有界饱和终结

只有以下任一有界触发成立，且 Explorer 被安全静默后，才允许评估 `bounded_saturation`：

- 探索的绝对时间预算到达；或
- `reachable_frontiers_stalled` 连续发生，并且原有恢复预算已经消费完。

运行时只使用本次 SLAM 图、机器人里程和 Action 账本，不读取 truth map、场景 YAML 区域或已知地图
大小。第二种触发用于修复“地图实际已经足够完整，但单次恢复只增长少量栅格便直接失败”的现场假阴性；
它不会降低原有 `40 cells / 0.2%` 收益门槛，也不会重置绝对 deadline。默认条件为：

- 最近至少 `2` 个连续低收益 epoch，每个 epoch 至少 `3` 个 terminal frontier goal；
- 建图阶段累计路径至少 `20 m`；恢复次数余量只记录诊断信息，不要求为耗尽预算制造动作；
- 残余 available frontier 作为 typed 诊断快照保留；hard-budget 不以其绝对 cluster 数单项否决，
  `reachable_frontiers_stalled` 路径仍要求 residual 为零；
- epoch 的单位终态目标增益没有同时达到 `40 cells` 与 `0.2%`；
- 有界触发后只做一次 360° final probe；其增益低于上述门槛；
- 地图连续静默至少 `15 s`；
- active/pending goal 为零、Action 总账排空；
- final probe 后执行新的 typed STOP，并观察到足够新的零速度。

任一证据不足都必须失败，不能退化为“时间到了就保存”或“第一次停住就保存”。该路径在生产侧只记录
中性的 `time_budget_exhausted` 或 `reachable_frontiers_stalled_bounded_saturation`；producer 写入的
`trigger_reason` 必须与任务终态逐字一致。是否达到项目地图质量门槛，仍由验收层独立裁决。

## 3. 真值隔离与验收层近似完成

生产进程允许读取：实时 `/scan`、`/odom`、TF、当前 `/map`、Action result 和任务配置中的通用阈值。
以下信息只允许进入 acceptance evaluator：真值地图、房间区域、Gazebo 真值位姿和整个场地的可达面积。

因此“近似完成”不是运行时自报覆盖率，而是两层证据同时成立：

1. 生产侧的 `SlamMappingCompletionEvidence` 证明饱和、账本排空、最终探测、停车和返航事务完整；
2. evaluator 使用真值独立验证原有地图门槛：总体可达覆盖 `>=90%`、每个可达区域 `>=85%`、
   reachable unknown `<=10%`、障碍召回 `>=60%`、false-free `<=5%`。

严格 frontier 终结与有界饱和终结共享相同地图质量、定位、路径安全、动态重规划和最终停车门槛；本轮不
降低 evaluator 阈值。历史 schema v4 PASS 只能作为旧实现基线，不能证明新收口与返航逻辑已经通过现场验收。

## 4. 安全静默与返航事务

硬预算触发时不能直接杀掉 Explore Lite，因为它可能仍持有 `NavigateToPose` UUID。收口顺序固定为：

```text
向 /explore/resume 发布 false
→ 等待 exploration_paused
→ 等待 active=0 且 accepted=terminal
→ 回收 explorer 进程
→ typed STOP
→ final probe + map quiet
→ 新 typed STOP + 新鲜零速度
```

起点在初始扫描之前从 `map→base_link` TF 动态捕获，不写入 YAML。探索收口后仍在 mapping stage 中执行
`ComputePathToPose` 准入和 `NavigateToPose` 返航；Action 成功后，还要用连续 TF 样本复核 XY/yaw 容差，
再发 typed STOP。只有返航成功、地图静默和回环尾帧结束后才保存地图；map_saver 返回后再次通过 typed
STOP 获取真实的新零速，才允许重启为 AMCL/Nav2。双确认避免同步存图耗时把返航时的零速证据拖旧。

返航证据必须证明：强类型 Action 成功、起终位姿同坐标系、XY/yaw 在容差内、返航发生在存图之前、
最终 `/cmd_vel` 新鲜且为零。任一条件缺失，任务 fail closed；禁止扩大 `max_cmd_vel_age_s` 或伪造时间戳。

## 5. 关键代码与接口

| 位置 | 职责 |
| --- | --- |
| `src/embodied_simulation/config/unknown_world_slam_mission.yaml` | 运行时饱和、返航和导航通用阈值；无场景坐标 |
| `exploration_saturation.py` | 跨 epoch 的峰值 known cells、terminal goal、路径与 final probe 纯领域判定 |
| `mapping_return.py` | 返航位姿、时间线、Action 终态和零速度纯领域契约 |
| `frontier_monitor.py` | 硬预算只发出 `assessment_required`，不伪造探索完成 |
| `mission_executor.py:_explore_with_bounded_recovery()` | frontier epoch、恢复、静默和饱和评估事务 |
| `mission_executor.py:_assess_time_budget_saturation()` | 时间预算触发后的 final probe |
| `mission_executor.py:_finalize_saturation_assessment()` | 两类有界触发共享的地图静默、STOP 与 fail-closed 判定 |
| `showcase_session_node.py:quiesce_frontier()` | 暂停 Explore Lite 并等待 Action 总账排空 |
| `showcase_session_node.py:run_mapping_return_goal()` | mapping stage 内返航、TF 复核与安全停车 |
| `SlamMappingCompletionEvidence.msg` | 饱和与返航的强类型生产证据 |
| `tools/acceptance/unknown_world_evidence.py` | 严格/近似完成与 evaluator-only 地图质量裁决 |
| `tools/acceptance/visual_runtime.py` | renderer/内存预检、D3D12 注入与双 GUI 安全降级 |
| `tools/acceptance/resource_watchdog.py` | 有界 JSONL 资源采样、低内存滞回与安全失败 |
| `tools/acceptance/runtime_log_health.py` | Nav2 Lifecycle 已关闭时快速终止外层等待 |

中文注释只解释真值隔离、Action 所有权、时间线和 fail-closed 风险，不逐行翻译代码。

## 6. 现场验收

先清理旧进程，再启动可视化正式入口：

```bash
CLEANUP_CONFIRM=true bash scripts/cleanup_simulation_processes.sh

HEADLESS=true USE_RVIZ=true \
  SLAM_NAV_PROGRESS_HEARTBEAT_S=10 \
  bash scripts/acceptance_test.sh unknown-world-slam-e2e
```

旧命令 `HEADLESS=false USE_RVIZ=true` 仍可运行；资源策略会在 8 GiB WSL 或软件渲染环境自动关闭
Gazebo 3D client，只保留 RViz。Gazebo server、物理、传感器和机器人运动不受影响。双 GUI 只用于短时
调试，并需显式设置 `SLAM_NAV_ALLOW_DUAL_GUI=true`。

现场观察必须同时满足：

1. 机器人从未知地图开始自主移动，没有播放固定房间路线。
2. 残余角落 frontier 长期低收益时，控制台明确进入 quiesce/final probe，而不是长时间无说明停住。
3. Explorer 收口后机器人回到开始建图时动态捕获的位置附近，再保存地图。
4. 界面切换到 AMCL/Nav2，完成 3 个本次地图采样目标和动态障碍重规划。
5. 任务正常结束、`/cmd_vel` 归零，控制台打印 PASS；不是由外层 timeout 或清理脚本杀进程。
6. 本次 session 的报告、地图、runtime log 和 manifest 都存在且时间一致。
7. 正式发布证据中的 `source_revision` 等于待发布提交，且 `source_dirty=false`；
   dirty run 可以用于开发诊断，但不能作为 `main` 的发布证据。

证据目录：

```text
logs/acceptance/unknown_world_slam_nav/<session_id>/
├── unknown_world_slam_e2e_report.json
├── unknown_world_map.yaml
├── unknown_world_map.pgm
├── runtime.log
├── resource_samples.jsonl
└── acceptance_session.json
```

本轮现场 session `20260725T120302Z-145519-3ec3df78` 正常运行 `990 s` 后自行结束：可达自由区覆盖
`99.75%`、最低区域覆盖 `98.34%`、AMCL P95 `0.154 m`，返航、3 个动态采样目标、动态重规划和最终
新鲜零速全部 PASS。资源 watchdog 共记录 199 个样本，会话 RSS 峰值 `2056.5 MiB`，最终 Swap 使用率
约 `0.01%`，无 `collision_monitor` 心跳故障。该次使用 strict final-confirmation 路径；残余前沿长期
存在时的 bounded saturation 路径仍由同一 typed 返航门槛和确定性测试覆盖。

真人语音联合入口在相同核心事务外再验证真实音频、VAD endpoint、WakeEvent 和 ASR；它不能替代上述
SLAM/Nav2 可视化验收。完整字段、阈值和故障定位见 [测试手册](../TESTING.md)。

## 7. Goal 完成条件

- 饱和、静默、返航、强类型证据和 evaluator 单元/集成测试通过。
- ROS 相关包构建与测试通过，公开 CLI 契约不回退。
- 新的可视化 `unknown-world-slam-e2e` 现场报告全部 checks/sections 为 true。
- 现场确认机器人先返航再存图，并继续完成 AMCL/Nav2/动态重规划。
- README、测试手册和学习笔记与实际实现一致。
- session `20260725T120302Z-145519-3ec3df78` 已满足上述现场条件；提交后仍需由 CI 复核轻量门禁。
