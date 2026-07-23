# 工程日志

## 2026-07-23：基线与计划

### 已完成

- 审计 `main`、`dev`、功能 PR 与 CI。
- Docker 交付 PR 合入 `dev`。
- unknown-world SLAM/Nav2 PR 在合并最新 `dev` 后通过：
  - repository layout；
  - ROS 2 build-and-test；
  - frontier patch replay；
  - container build/runtime smoke。
- unknown-world SLAM/Nav2 PR 合并到 `dev`，合并提交：`68a4513`。
- 从该提交创建 `feature/demo-control-plane`。
- 建立本目录，保存演示目标、架构决策、资料、工具和风险。

### 正在执行

- 严格 E2E 的 ROS 包版本混装问题已排除，并通过最小 Nav2/Gazebo 运动烟测。
- 第二次 E2E 在首次 `reachable_frontiers_stalled` 恢复处误拒绝；诊断地图实际
  达到 99.78% 总覆盖、98.51% 最差区域覆盖。
- 已从 `origin/dev@68a4513` 创建 `fix/frontier-saturation-completion`，
  以连续低收益 epoch 修复完成判定，不降低原 40 cells / 0.002 阈值。
- `feature/demo-control-plane` 已形成 typed authority、C++ 键盘、自治 mux、
  最终速度 AuthorityGate、ActionGuard 准入和 SLAM 接管取消的首版实现。

### 待执行

- 完成两个分支的本地门禁；先合入严格验收稳定性修复，再让控制面分支同步最新 `dev`。
- 严格 E2E 通过后创建 `dev -> main` 发布 PR。

### 证据

- PR #85：<https://github.com/Edddddddddy/embodied_agent_ws/pull/85>
- 最终严格 E2E 报告路径将在完成后追加。

## 2026-07-23：可达前沿收口与控制面安全门

- PR #87（`e9d52d3`）完成 `reachable_frontiers_stalled` 的有界饱和收口。
- 本地阶段门禁：596 项 pytest、283 项 colcon 测试通过。
- 审计旧成功 session 时发现报告顶层 `final_phase`、`mapping_path_m`、
  `frontier_goal_count` 为 `null`，且无法定位产生证据的源码版本。
- 修复分支已补：
  - `source_revision`、`source_dirty`；
  - `final_phase`、`mapping_path_m`、`frontier_goal_count` 三项顶层诊断字段。
- 多输入控制面新增：
  - manager heartbeat lease 与重启 `manager_epoch`；
  - Nav2/voice 自治 mux；
  - AUTONOMY/KEYBOARD typed 正授权；
  - TurtleBot3 平面轴与速度包络；
  - 稳定窗口逐帧验收，未授权非零速度零容忍；
  - 键盘 deadman 与非阻塞单实例锁。

## 2026-07-23：严格基线证据与统一演示安排

- 在干净 detached worktree 上对 `origin/dev@a1b6badae7ca688007ef0aa176c750d483750f29`
  运行 `unknown-world-slam-e2e`，session：
  `20260723T130648Z-325489-748996e5`。
- 严格门禁 1002 秒通过：
  - 总覆盖率 `0.999`，最弱区域覆盖率 `0.992`；
  - 建图路径 `160.543 m`，共完成 `39` 个 frontier goal；
  - 动态起点返航、同会话存图、AMCL/Nav2 lifecycle、3 个动态采样目标、
    动态障碍重规划和最终新鲜零速全部通过；
  - AMCL P95 位置误差 `0.185 m`；
  - 报告绑定 exact SHA，`source_dirty=false`；
  - session manifest 标记 `cleanup_complete=true`，无遗留 ROS/Gazebo 进程。
- 发布 PR #88 已合入 `main`，merge commit：
  `4125d4b5fba59fdb5f44700357cd8e32e4ffda16`。
- `main` 的 ROS 2 CI、容器交付均通过；阶段标签 `v0.5.0` 已推送。
- tag workflow 对 exact candidate digest 完成 runtime smoke，再发布到 GHCR，
  immutable release evidence 与 manifest 上传成功。
- 控制平面未完成改动未混入该发布版本。
- 统一演示开发拆为三个完整功能 PR：
  1. session 级控制权、键盘接管、速度安全与同代终止确认；
  2. 同一 Gazebo/RViz/Agent 会话中的 mapping→navigation 阶段切换；
  3. `showcase offline|online` 单一入口、状态看板、证据报告与文档。
- 明确 launch 所有权：顶层编排归 `embodied_agent_bringup`，仿真、Agent、
  SLAM/Nav2 和控制权分别保留自己的窄 interface。

## 2026-07-23：控制平面真实 ROS stage 收口

- 独立审查发现并修复三项提交前 P1：
  - Python/C++ authority lease 统一为同 epoch 断租后 sticky fail-closed；
  - 每次 RESUME 后先完成新一代零速握手，再接受自治非零速度；
  - manager 与共享 launch factory 默认不预先确认 quiescence，单独重启不能
    绕过旧 Nav2/Explore terminal 证据。
- 进一步关闭独立入口的可用性/安全冲突：
  - 纯 C++ 状态机默认同样改为严格 bootstrap；
  - mapping/navigation 阶段 launch 不再创建 manager；
  - 缺少会话级 coordinator 却请求自带 manager 时启动前 fail-fast；
  - `continuous_nav2_voice_control.sh` 开启 Gate 时会等待两个 typed manager
    service及 `/slam/session_state` coordinator，避免系统运行后才表现为永久 HOLD。
- 处理 cold-start DDS late-join 竞态：会话根在确认没有旧 Gazebo/Nav2/SLAM
  进程和旧 manager 后，显式声明“整套运动栈冷启动”并消费一次 bootstrap ACK；
  因而不会在 ActionGuard 尚未订阅时发布一次性 STOP。以后所有撤权仍必须走
  Explore/Nav2 terminal、exact priority STOP result 与新鲜零速的严格 ACK。
- 新增 `control_authority_bootstrap` typed 客户端：校验初始
  `HOLD/seq=0/ack=true`，调用 `RESUME_AUTONOMY`，再等待相同 manager epoch 的
  AUTONOMY topic 状态。隔离 domain 的真实 manager 烟测通过：
  `epoch=30382631394719, sequence=1`。
- manager 生成的 priority STOP ID 加入 `manager_epoch`，避免重启后与旧调度器
  幂等键冲突。
- Nav2 STOP 语义从“cancel request 已接受”收紧为等待真实 Action terminal；
  cancel 拒绝和 terminal timeout 均 fail-closed。
- 拒绝 quarantine 帧的日志改为节流告警，避免正常安全隔离刷屏淹没故障。
- 低内存门禁结果：
  - repository + bringup + SLAM/launch + release-gate Python：
    `649 passed`（包含 cold-start typed bootstrap 的 10 个新增用例）；
  - `embodied_agent_cpp`：15 个 CTest suite 全部通过；
  - `embodied_simulation`：14 个 CTest suite 全部通过；
  - `control-authority-stage` 通过真实 manager、mux、Gate、typed ACK、
    lease 失效与 manager 重启验证；
  - `continuous-multi-command` 在线、离线回归均通过。
- WSL 构建策略改为单线程、精确包选择；不再用会拉入整套 Agent 的
  `--packages-up-to embodied_simulation`。
