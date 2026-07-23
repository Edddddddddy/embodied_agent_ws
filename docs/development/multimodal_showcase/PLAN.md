# 统一演示开发计划

## 1. 最终目标

在一个持续运行的 Gazebo/RViz 场景中，用同一台 TurtleBot3 演示：

1. 键盘手动控制与松键自动停车。
2. 离线或在线语音的单命令、多命令与急停。
3. 未知环境前沿探索、SLAM 建图、返航和地图保存。
4. 切换到 AMCL/Nav2 后的语音目标点导航、巡航和动态避障。
5. 自动任务执行中键盘安全接管，随后由用户显式恢复。
6. 结束时输出可审计证据，并证明最终 `/cmd_vel` 为新鲜零速。

本计划不重写已经通过严格验收的 SLAM 状态机。新控制平面负责控制权和速度安全；`SessionOrchestratorNode` 继续是建图/导航阶段的唯一业务状态机。

当前实施状态：

| 批次 | 状态 | 对外能力 |
|---|---|---|
| 稳定基线 `v0.5.0` | 已发布 | 严格未知世界 SLAM→存图→定位→Nav2→动态重规划 |
| 第一轮 `feature/demo-control-plane` | 本地门禁已通过，待 PR | 键盘/自治互斥、急停、deadman、typed ACK、速度安全 |
| 第二轮 `feature/demo-persistent-session` | 计划中 | Gazebo/机器人/Agent 不重启的建图→导航切换 |
| 第三轮 `feature/demo-showcase-ux` | 计划中 | `showcase offline|online`、状态栏、统一证据与讲稿 |

## 2. 非目标

- 不在第一轮实现真实硬件 UART/SPI。
- 不用键盘节点复制 Nav2、ActionGuard 或 SLAM 状态机。
- 不把 `twist_mux` 当作业务状态机；它只做速度源优先级与超时。
- 不用预生成地图伪装成同一会话的实时建图结果。
- 不把快速现场演示的 PASS 写成严格 unknown-world 发布验收 PASS。

## 3. 目标架构

统一演示的顶层 launch 归属 `embodied_agent_bringup`，不再把 Agent、
SLAM/Nav2 和控制权参数继续堆入仿真包。顶层 interface 只保留
`agent_mode`、`profile`、`use_rviz` 和 `headless`；VAD、速度限制、Nav2、
SLAM 等实现参数由各模块配置文件管理。

```text
麦克风 -> VAD/ASR -> NLU/队列 -> RobotCommand -> ActionGuard
                                                |
                                                v
                                       ROS 2 Action/BT
                                                |
                        +-----------------------+----------------------+
                        |                                              |
                  语音原语速度                                   Nav2 Goal
        /control/voice/cmd_vel                           controller_server
                        |                                      |
                        +------------> twist_mux <--------------+
                                         |
                              /control/autonomy/cmd_vel
                                         |
键盘 -> KeyboardTeleop -> /control/keyboard/cmd_vel      typed authority state
                        \                 |                 /
                         +------> C++ VelocityAuthorityGate
                                         |
                              /control/selected/cmd_vel
                                         |
                                Nav2 Collision Monitor
                                         |
                                      /cmd_vel
                                         |
                                  Gazebo TurtleBot3
```

控制权状态通过 typed ROS 2 消息发布。业务节点根据状态取消或拒绝任务；速度安全末端保持单一。

运行时按所有权拆分：

- 常驻运行时：Gazebo、机器人实体、RViz、Agent、控制权 manager、速度安全管线和会话编排器。
- 建图阶段：SLAM Toolbox、前沿探索器和探索所需的 Nav2 节点。
- 导航阶段：本次会话地图的 map server、AMCL 和导航 Nav2 节点。

建图切换到导航时只替换“阶段节点”。常驻进程 PID、控制权
`manager_epoch`、Gazebo 实体和 Agent 会话必须保持不变。

## 4. 分支与迭代

### 4.1 基线收口

分支：已完成的功能 PR → `dev` → 发布候选。

门槛：

- PR 仓库、ROS 2、前沿探索回放、容器测试全绿。
- 在最终 `dev` SHA 上重新运行 `unknown-world-slam-e2e`。
- 通过 `dev -> main` PR 发布，不直接推送 `main`。

### 4.2 第一轮：控制平面

分支：`feature/demo-control-plane`

交付：

- `ControlAuthorityState.msg` 与 `SetControlAuthority.srv`。
- 纯 C++ `ControlAuthority` 状态机。
- C++ 键盘节点：W/S/A/D、Space、X、R、Q。
- 600 ms 默认 deadman、锁存急停、显式恢复、防幽灵运动。
- `twist_mux` 只仲裁 Nav2/语音自治源，C++ Gate 对自治/键盘做正授权。
- Collision Monitor 成为唯一最终 `/cmd_vel` 发布者。
- control authority manager 是 session 级唯一节点，不能随 mapping/navigation 阶段重启。
- 接管后聚合 Explore/Nav2 terminal、priority STOP result 和新鲜零速证据；
  只有相同 manager epoch 与 revocation sequence 的 typed ACK 才允许恢复自治。
- 自动任务在人工接管或急停时 fail-closed 取消。
- `control-authority-stage` 自动验收。

自治速度默认优先级与控制权：

| 来源/状态 | 优先级或授权 | 说明 |
|---|---:|---|
| 语音原语 | mux 70 | move/turn/arc 等 Action 执行 |
| Nav2/探索 | mux 50 | 自动建图、导航与巡航 |
| AUTONOMY | 只通过自治 mux | 键盘速度即使仍在发布也被归零 |
| KEYBOARD | 只通过键盘 | 约 600 ms deadman，旧自治速度不能复用 |
| HOLD / ESTOP / manager 失联 | 持续零速 | fail-closed，不依赖一次性 STOP |

第一轮的恢复语义是“取消旧自动事务后允许发起新任务”，不承诺断点续跑旧 frontier goal。

### 4.3 第二轮：持久会话

分支：`feature/demo-persistent-session`

交付：

- Gazebo、机器人、Agent 和 RViz 在演示期间保持运行。
- 建图阶段只切换 SLAM/Explore，导航阶段只切换 map_server/AMCL/Nav2。
- `StageProcessManager` 对编排层提供
  `start_base()`、`start_mapping()`、`switch_to_navigation(map_path)` 和
  `shutdown()` 四个高层操作，隐藏子进程和 readiness 细节。
- 保存本次地图后校验时间戳、hash 和 session provenance。
- 建图到定位的有界切换与失败回滚。
- 人工接管时暂停/取消当前任务；显式恢复后由编排层决定重试或新任务。
- `showcase-gazebo-e2e` 验证同一场景的完整阶段转换。

### 4.4 第三轮：展示体验与证据

分支：`feature/demo-showcase-ux`

> 以下 `showcase*` 命令属于第三轮计划接口，当前尚未注册到
> `acceptance_test.sh`，不能作为本轮控制平面 PR 的验收命令。

公开入口保持一个：

```bash
bash scripts/acceptance_test.sh showcase offline
bash scripts/acceptance_test.sh showcase online
```

内部入口：

```bash
bash scripts/acceptance_test.sh showcase-stage
bash scripts/acceptance_test.sh showcase-gazebo-e2e
```

交付：

- `showcase_quick`：现场 12–15 分钟可讲解流程。
- `unknown_world_strict`：发布前完整质量门禁。
- 统一状态栏：会话阶段、控制权、队列、Nav2 goal、地图增长、急停。
- 单份结构化报告，包含 voice、keyboard、authority、slam、navigation、safety。
- 精简 README、演示讲稿和代码走读地图。

键盘需要独占 TTY，第一版保留第二终端：

```bash
bash scripts/keyboard_control.sh
```

它只负责发布键盘输入与控制权请求，不得启动第二套机器人或仿真系统。

## 5. 现场演示流程

系统冷启动在演示前完成。

| 时间 | 演示内容 | 关键证据 |
|---|---|---|
| 0:00–1:00 | Gazebo/RViz 与 readiness | 单场景、控制权 HOLD |
| 1:00–2:00 | WASD 手动控制 | 非零速度、deadman 零速 |
| 2:00–3:00 | 语音多命令 | endpoint、ASR、NLU batch、typed Action |
| 3:00–6:30 | 自动建图 | frontier、地图增长、机器人运动 |
| 中途 | 键盘接管与 R 恢复 | cancel→STOP→KEYBOARD→HOLD→AUTO |
| 6:30–8:30 | 收敛、返航、存图、AMCL | 本次地图 provenance |
| 8:30–10:30 | 语音目标点导航 | Nav2 result、定位质量 |
| 10:30–12:00 | 导航中再次接管 | 旧 generation 不再输出 |
| 12:00–13:30 | 动态障碍与 X 急停 | replan、队列清空、锁存零速 |
| 13:30–15:00 | Q 安全退出与报告 | cleanup、最终新鲜零速 |

## 6. 验收分层

### L0：纯单元测试

- 键位映射、限幅、deadman。
- 控制权状态转换、急停锁存、RESET 与 RESUME 分离。
- 边沿只生成一次 priority STOP。
- ActionGuard 在 KEYBOARD/ESTOP 下拒绝普通动作。
- 旧 transition generation 不可恢复运动。

### L1：ROS stage

- `control-authority-stage` 使用真实 manager、mux 和 gate，验证
  `AUTONOMY -> KEYBOARD -> HOLD -> AUTONOMY`、租约、同代 typed ACK 与急停。
- 后续 `showcase-stage` 再加入 fake Explore/Nav2 owner，验证 Action terminal、
  priority STOP 和新鲜零速全部到齐后才允许恢复。
- 两个 stage 都走生产 topic/service，不直接调用状态机内部实现。

### L2：控制速度管线

- voice/Nav2 先进入 `twist_mux`，键盘不参与业务优先级竞争。
- `VelocityAuthorityGate` 按 typed authority 与 steady-clock 租约选择自治或键盘。
- selected velocity 再进入 Collision Monitor。
- 最终 `/cmd_vel` 只有安全末端发布。

### L3：快速 Gazebo E2E

- 同一场景完成键盘、语音、地图增长、存图、AMCL、至少一个目标、接管和急停。
- 目标机器运行时间不超过 10 分钟。

### L4：严格发布验收

- 保留现有 `unknown-world-slam-e2e` 门槛。
- 任何 frontier、地图、定位或目标采样策略变更都必须重跑。

### L5：真人语音

- `offline` 必跑，`online` 补充。
- synthetic 输入不能替代真实音频、endpoint、唤醒和 ASR 证据。

## 7. 场景矩阵

状态列中的“计划”表示入口尚未注册；当前可执行入口应始终以
`bash scripts/acceptance_test.sh --help-all` 为准。

| 状态 | 场景 | 输入 | 运行时 | 验证目标 |
|---|---|---|---|---|
| 已实现 | `control-authority-stage` | synthetic velocity + virtual authority requests | 最小 ROS graph | 控制权、租约、typed ACK 与急停 |
| 计划 | `showcase-stage` | synthetic | mock/stage | 建图、导航、接管状态机 |
| 计划 | `showcase-gazebo-e2e` | synthetic | `showcase_apartment` | 可重复的完整演示门禁 |
| 计划 | `showcase offline` | 真人麦克风 + 键盘 | 持久 Gazebo/RViz | 主要现场演示 |
| 计划 | `showcase online` | 在线 Agent + 键盘 | 持久 Gazebo/RViz | 在线链路补充 |
| 已实现 | `unknown-world-slam-e2e` | synthetic | 严格未知世界 | 发布质量门禁 |
| 已实现 | `voice-unknown-world-slam-e2e` | 真人语音 | 严格未知世界 | 真人联合严格证据 |

## 8. 快速与严格证据边界

`showcase_quick` 使用 `evidence_kind=multimodal_showcase_quick`。严格完整链路使用现有 unknown-world schema。

如果现场快速建图未完成：

- 可以明确切换到 `prevalidated_map_replay` 继续讲导航。
- 报告必须写入 `degraded=true`、`same_session=false`。
- 不得把旧地图写成“本次建图生成”。

## 9. 旧入口退役规则

- `core`、`robotics-gate`、`unknown-world-slam-e2e` 和
  `voice-unknown-world-slam-e2e` 保留为稳定门禁。
- `gazebo`、`nav2-stage`、`slam-nav-e2e` 保留为开发诊断入口，不进入主演示流程。
- 旧的 mapping/save/navigation 脚本先改为新模块的薄 wrapper；一个版本内无调用者后再删除。
- `smoke_test_*` 不批量删除。先将断言迁入 `tools/acceptance` 的场景/探针，
  再根据注册表与全文搜索证明“零调用者”，分批清理。
- 最终 `scripts/` 只保留用户入口和环境准备；测试实现归入 `tools/acceptance`。

## 10. 完成定义

一轮功能只有在以下事项全部满足后才进入 GitHub CI：

1. 单元测试和相关回归通过。
2. 新接口、参数、QoS 和失败语义有文档。
3. 关键 C++/Python 逻辑有解释原因的中文注释。
4. 本目录追加工具、踩坑、决策和测试证据。
5. 工作区干净，提交按功能拆分。
6. PR 指向 `dev`，CI 全绿后合并。
