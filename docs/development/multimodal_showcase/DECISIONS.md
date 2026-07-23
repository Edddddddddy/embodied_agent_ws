# 架构决策

## ADR-001：控制权状态机与速度 mux 分离

状态：已接受。

决定：

- C++ `ControlAuthorityManager` 管理控制权、接管、急停和显式恢复。
- `twist_mux` 只按优先级和 timeout 仲裁 Nav2/语音两个自治速度源。
- C++ `VelocityAuthorityGate` 按 typed authority 对自治/键盘做互斥正授权。
- HOLD、ESTOP、来源超时或 manager 心跳超时均持续输出零速。

原因：

- mux 不知道 Nav2 Action、语音队列或 SLAM 会话，不能正确取消业务任务。
- 自写自治速度 mux 会重复成熟组件的 QoS、priority 和 timeout 行为。
- `twist_mux` 的全局 priority lock 无法表达“KEYBOARD 时只允许键盘、
  AUTONOMY 时只允许自治”的 allowlist，因此不能承担安全授权。
- 分离后，状态机和最终权限门都可纯 C++ 单测，自治仲裁可独立集成测试。

## ADR-002：Collision Monitor 是唯一最终速度出口

状态：已接受。

决定：

- Nav2/语音先进入 mux，mux 与键盘进入 C++ AuthorityGate。
- Gate 输出进入 Nav2 Collision Monitor。
- 只有 Collision Monitor 向 TurtleBot3 bridge 的 `/cmd_vel` 发布。

原因：

- 如果键盘或语音绕过 Collision Monitor，人工控制可能绕过碰撞安全。
- 如果多个节点直接发布 `/cmd_vel`，DDS 不提供业务优先级，结果不可证明。

## ADR-003：人工接管后必须显式恢复

状态：已接受。

决定：

- 键盘非零输入触发 takeover。
- deadman 只负责零速并进入 HOLD，不自动回到 AUTONOMY。
- 急停 RESET 只解除锁存，仍保持 HOLD；用户按 R 才恢复自动控制。

原因：

- 避免用户松键后旧 Nav2 goal 或旧语音动作突然恢复，形成 ghost motion。
- 把恢复动作变成可观察、可审计的用户决定。

## ADR-004：不建立第二套 SLAM 状态机

状态：已接受。

决定：

- `SessionOrchestratorNode` 继续唯一拥有建图→返航→存图→定位→导航流程。
- 控制权节点只发取消/STOP 和 typed authority state。

原因：

- 两个 FSM 同时拥有阶段生命周期会产生竞态和难以复现的恢复错误。
- 现有 unknown-world 严格验收已经证明原状态机可用，应在其边界上扩展。

## ADR-005：现场快速证据与严格发布证据分离

状态：已接受。

决定：

- 现场使用 `showcase_quick`。
- 发布使用 `unknown_world_strict`。
- 预生成地图降级必须显式标记。

原因：

- 当前严格 E2E 约 18 分钟，仅机器运行就超过部分现场预算。
- 为赶时间降低门槛会破坏证据可信度。

## ADR-006：第一轮取消旧自动事务，不实现断点续跑

状态：已接受，第二轮复审。

决定：

- 人工接管或急停取消当前 Explore/Nav2 事务。
- 恢复后允许发起新任务，不自动续跑旧 goal。

原因：

- 安全取消可以复用已有 Scheduler、typed STOP 和会话 cleanup。
- 真正暂停需要冻结 deadline、goal generation、地图阶段和恢复点，不能混入控制平面最小迭代。

## ADR-007：权限通过不等于速度合法

状态：已接受。

决定：

- `VelocityAuthorityGate` 同时执行来源授权与 TurtleBot3 速度包络校验。
- 只允许 `linear.x` 和 `angular.z`；其余四个轴出现非零值时拒绝整条消息并归零。
- 默认限制为 `|linear.x| <= 0.26 m/s`、`|angular.z| <= 1.82 rad/s`，
  可通过 ROS 参数缩小。
- manager 使用 heartbeat lease 与 `manager_epoch`；进程重启只能先以
  `seq=0/HOLD/initialized` 建立新 epoch。
- 同一 epoch 一旦越过 lease 即保持 fail-closed；迟到 heartbeat 不能重新授权，
  必须由新 manager epoch 从 HOLD 恢复。

原因：

- 上游坐标系或消息类型错误不能靠单轴截断“修成”另一条合法运动。
- 权限状态仍正常时，越界或非平面速度也必须 fail-closed。
- 单独的 epoch 能区分重启后的序号归零和旧进程迟到消息。

## ADR-008：控制权 manager 必须跨阶段持久

状态：已接受。

决定：

- `control_authority` 由统一 showcase session 启动，全程只有一个实例。
- mapping/navigation 子阶段不得自行启动 manager。
- 阶段切换后必须保持相同 `manager_epoch`；manager 异常退出时整场会话
  fail-closed，不在原地隐式重建自治权限。

原因：

- 子阶段重启 manager 会把 HOLD 重置成新的初始状态，使旧 Explore/Nav2 速度
  有机会在阶段切换后重新获得权限。
- 控制权属于整场人机交互会话，不属于任一 SLAM 或 Nav2 子进程。

## ADR-009：恢复自治需要同代聚合终止确认

状态：已接受。

决定：

- 离开 AUTONOMY 时生成 `autonomy_revocation_sequence`。
- 编排层聚合 Explore/Nav2 terminal、priority STOP result 和停止请求后的
  新鲜零速证据。
- 只有 `manager_epoch + autonomy_revocation_sequence` 完全匹配的 typed ACK
  才允许执行 `RESUME_AUTONOMY`。

原因：

- 固定 sleep、进程退出或某一时刻观察到零速，都不能证明旧 Action 已进入
  terminal 状态。
- 同代 ACK 将“旧任务已终止”和“允许新自治任务”建立因果关系，能阻止
  late result、旧 heartbeat 或旧速度造成 ghost motion。

## ADR-010：manager 启动默认不等于自治已经静默

状态：已接受。

决定：

- `control_authority` 的运行时参数
  `bootstrap_quiescence_acknowledged` 默认值为 `false`。
- manager 无论是首次启动还是单独重启，都先停在 HOLD；必须由会话编排器发送
  当前 `manager_epoch + revocation_sequence` 的 typed ACK 才可恢复自治。
- 只有能证明“整个运动栈与 manager 同时冷启动、没有旧 goal 和旧速度”的顶层
  集成测试，才允许显式覆盖为 `true`。
- 当前 `voice_slam_nav_showcase.sh auto` 在启动任何运动节点前先运行残留进程检查，
  拒绝已存在的 authority service，随后才以 `true` 创建本会话唯一 manager；
  manager 异常退出时不自动 respawn。

原因：

- manager 进程重启不代表旧 Explore/Nav2 Action 已终止。
- Gate 的零速隔离能阻止旧帧立即复驶，但不能代替 Action terminal 证据。
- 安全默认值应在节点和共享 launch factory 两层保持一致，避免某个独立入口
  无意中绕过会话级静默协议。
