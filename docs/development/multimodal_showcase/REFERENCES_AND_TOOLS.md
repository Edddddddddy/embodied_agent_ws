# 多模态演示参考资料与工具

本文件记录本轮实际用到的资料、代码入口和诊断命令；规划性资料不写成已集成能力。

## 1. 上游资料

- Navigation2：<https://github.com/ros-navigation/navigation2>
- BehaviorTree.CPP：<https://github.com/BehaviorTree/BehaviorTree.CPP>
- Nav2 Lifecycle Manager：
  <https://docs.nav2.org/configuration/packages/configuring-lifecycle.html>
- Nav2 AMCL：
  <https://docs.nav2.org/configuration/packages/configuring-amcl.html>
- Nav2 Collision Monitor：
  <https://docs.nav2.org/tutorials/docs/using_collision_monitor.html>
- Nav2 Velocity Smoother：
  <https://docs.nav2.org/configuration/packages/configuring-velocity-smoother.html>
- `twist_mux`：<https://github.com/ros-teleop/twist_mux>
- ROS 2 Managed Nodes：
  <https://design.ros2.org/articles/node_lifecycle.html>
- ROS 2 QoS：
  <https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html>
- Linux `/proc/<pid>/stat`：
  <https://man7.org/linux/man-pages/man5/proc_pid_stat.5.html>

使用原则：

- 复用 Nav2/SLAM Toolbox/twist_mux 的成熟职责，不复制上游仓库结构。
- lifecycle、QoS 和 Action 以官方语义为准，不能用固定 sleep 模拟 readiness。
- Linux procfs 只用于不可逆身份摘要；报告不保存原始 argv 或凭据。

## 2. 本轮关键代码入口

| 领域 | 文件 | 作用 |
|---|---|---|
| 会话 FSM | `src/embodied_slam_tools/embodied_slam_tools/showcase_session_node.py` | HOLD/ACK/阶段切换、Lifecycle、AMCL 初始化与 fail-close |
| 进程所有权 | `src/embodied_slam_tools/embodied_slam_tools/stage_process_manager.py` | base/stage/explorer/map-saver 与 owned process tree |
| 探索完成 | `src/embodied_slam_tools/embodied_slam_tools/frontier_monitor.py` | hard budget、低收益恢复与完成判定 |
| 最终确认 | `src/embodied_slam_tools/embodied_slam_tools/mission_executor.py` | 近期仍有增益时的一次 240 s 有界确认 |
| 常驻层 | `src/embodied_simulation/launch/persistent_voice_nav_base.launch.py` | Gazebo/RSP/Agent/Nav2 common/速度安全 |
| 建图层 | `src/embodied_simulation/launch/persistent_mapping_stage.launch.py` | SLAM provider 与 mapping executor |
| 导航层 | `src/embodied_simulation/launch/persistent_navigation_stage.launch.py` | map server/AMCL/Nav2 executor/tracker |
| 入口编排 | `scripts/voice_slam_nav_showcase.sh` | 会话准备与进程启动边界 |
| AMCL helper | `scripts/publish_nav2_initial_pose.py` | 独立诊断；无 subscriber 时 fail |
| 运行时身份 | `tools/acceptance/runtime_identity.py` | `/proc` 身份采样与角色选择 |
| 连续性判定 | `tools/acceptance/runtime_continuity.py` | checkpoint schema、顺序和身份比较 |
| E2E 包装 | `tools/acceptance/scenarios/showcase_gazebo_e2e.py` | strict 场景上附加 persistent 门禁 |
| evidence | `tools/acceptance/showcase_evidence.py` | 从原始报告重算最终结论 |

typed 主边界仍位于 `src/embodied_agent_interfaces/`：
`ManageSlamSession.action`、`ExecuteRobotCommand.action`、
`SetControlAuthority.srv`。

## 3. 版本控制与 GitHub

```bash
git worktree list --porcelain
git status --short
git diff --stat
git diff --check
git branch -vv

gh auth status
gh pr checks
gh run view
```

流程：`feature/* -> dev -> main`。先在本地完成一个功能闭环，再 push 和创建 PR；
不得直接推 `main`。当前 persistent 分支 heavy PASS 前不进入 PR。

## 4. 构建与测试

低内存 WSL 推荐：

```bash
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build \
  --executor sequential \
  --packages-select embodied_slam_tools embodied_simulation

colcon test \
  --packages-select embodied_slam_tools embodied_simulation \
  --event-handlers console_direct+
colcon test-result --verbose

pytest -q src/embodied_slam_tools/test tests/repository \
  tests/integration/slam_nav/test_continuous_nav2_voice_control_script.py
python3 -m compileall -q src/embodied_slam_tools/embodied_slam_tools tools/acceptance
```

本轮执行结果：组合 pytest `717 passed`；两个 ROS 包最新共 `692 tests`，零失败。

## 5. ROS/Gazebo 现场诊断

重型门禁前：

```bash
CLEANUP_CONFIRM=true bash scripts/cleanup_simulation_processes.sh
free -h
swapon --show
```

最常用的运行时检查：

```bash
ros2 lifecycle get /typed_action_bridge
ros2 lifecycle get /collision_monitor
ros2 topic info -v /cmd_vel
ros2 topic hz /cmd_vel
ros2 topic echo --once /cmd_vel
ros2 param get /collision_monitor stop_pub_timeout
ros2 topic info -v /initialpose
ros2 topic info -v /amcl_pose
```

诊断原则：

- 使用隔离的 `ROS_DOMAIN_ID` 和 `GZ_PARTITION`，避免旧会话污染。
- `/cmd_vel` 必须只有 Collision Monitor 作为生产发布者。
- 检查 lifecycle 最终状态，不把 service 请求成功当成 ACTIVE。
- `/initialpose` 必须先有匹配 subscriber；之后再等当前 generation 的
  `/amcl_pose`。
- 通过 `/proc/<pid>/{stat,exe,cmdline}` 与 boot ID 识别进程，不只看 PID。

## 6. 权威验收与证据

当前分支复现命令：

```bash
HEADLESS=true USE_RVIZ=false SLAM_NAV_PROGRESS_HEARTBEAT_S=15 \
  bash scripts/acceptance_test.sh showcase-gazebo-e2e
```

fresh session `20260724T053935Z-1431080-d6efcab6` 已通过该命令，生成：

```text
logs/acceptance/showcase_gazebo_e2e/20260724T053935Z-1431080-d6efcab6/
├── showcase_gazebo_e2e_report.json
├── acceptance_session.json
└── runtime.log
```

这是本机可复查但不会进入 Git 的运行产物；不要把它写成 GitHub 上必然失效的相对
链接。

它记录 coverage `0.998`、最弱区域 `0.985`、AMCL P95 `0.125 m`、3 个导航目标、
路径 `147.661 m`、32 个 frontier goal、动态重规划、runtime continuity 和最终
零速全部通过。报告同时标明 `source_dirty=true`，所以它是当前 worktree 的验收
事实，不是已发布版本。

2026-07-21 的报告继续只作为修改前 strict 基线引用：

```text
/home/ubuntu/embodied_agent_ws_worktrees/voice-unknown-world-e2e/
logs/acceptance/unknown_world_slam_nav/
20260721T072342Z-2344751-5452a492/
```

该旧基线是 mock provider，不是 persistent fresh evidence，也不是麦克风验收。

## 7. Shell 使用规范

- Windows 发起 WSL：优先绝对路径和短命令。
- Bash 变量、管道和 here-doc：写入版本化脚本后执行。
- 文件搜索：优先 WSL 原生 `rg`，异常时确认 PATH 或临时使用 `/usr/bin/grep`。
- 文件编辑：在一个工作树内完成，提交前用 `git status --short` 核对没有跨分支
  或跨 worktree 写入。
- 更完整的跨 shell 规范见 `docs/development/WSL_POWERSHELL.md`。
