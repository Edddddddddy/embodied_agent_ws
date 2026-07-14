# 学习笔记：关键技术点与设计取舍

这份笔记面向复盘和面试讲解。每个技术点都按四个问题组织：

- 关键代码在哪里？
- 这一层怎么设计？
- 为什么这样设计？
- 和其他方案相比有什么区别？

如果只想先看主链路图，优先打开
[FINAL_ARCHITECTURE_DIAGRAMS.md](FINAL_ARCHITECTURE_DIAGRAMS.md)。
如果要准备面试代码走读，优先打开
[VOICE_TO_SIMULATION_CODE_WALKTHROUGH.md](VOICE_TO_SIMULATION_CODE_WALKTHROUGH.md)，
它按“语音输入 → Agent → ActionGuard → ROS 2 Action → Gazebo/Nav2 执行”列出了关键文件、
关键函数和上下游接口。

## 1. ROS 2 通信模型：topic、msg、action 的分工

关键代码：

- `src/embodied_agent_interfaces/msg/RobotCommand.msg`
- `src/embodied_agent_interfaces/msg/RobotCommandFeedback.msg`
- `src/embodied_agent_interfaces/msg/RobotCommandResult.msg`
- `src/embodied_agent_interfaces/action/ExecuteRobotCommand.action`
- `src/embodied_agent_cpp/src/typed_action_bridge_node.cpp`
- `src/embodied_agent_cpp/src/typed_action_bridge_main.cpp`
- `src/embodied_agent_cpp/CMakeLists.txt`
- `src/embodied_agent_cpp/src/typed_action_demo_client.cpp`
- `src/embodied_agent_cpp/include/embodied_agent_cpp/typed_action_client_contract.hpp`
- `scripts/audit_cpp_action_reports.py`
- `src/embodied_simulation/src/simulation_control_node.cpp`

设计方式：

- Agent 先把 LLM 或 fallback parser 的领域动作映射为强类型 `RobotCommand` candidate。
- C++ ActionGuard 对 candidate 做白名单、字段约束和限幅，再发布受信任的 `RobotCommand`。
- C++ `ActionScheduler` 将受信命令排成单执行槽 FIFO，再由 typed action bridge 发送为 `ExecuteRobotCommand` goal。
- `embodied_agent_cpp` 不增加 ROS 包，而是在 CMake 内拆成 control/audio/hardware 三个库；节点只链接
  自己需要的模块。typed bridge 同时提供 component 与原可执行入口，并由 Lifecycle manager 激活。
- bridge 把命令、Action 回调和诊断分到互斥/reentrant callback group；deactivate 通过
  `ActionScheduler.clear_all()` 取消活动 goal、清空 FIFO 并发布可关联的 canceled result。
- `typed_action_demo_client` 是面试/调试用最小 C++ action client：从命令行构造
  `RobotCommand`，直接发送 action goal，打印 feedback/result 并用结果决定进程退出码；
  它还支持定时取消、结果等待超时和 `CPP_ACTION_REPORT` 结构化报告。
- simulation executor 返回 feedback/result，并驱动 `/cmd_vel`。
- 终态契约把 `STATUS_SUCCEEDED/CANCELED/TIMED_OUT/BLOCKED` 映射成稳定业务结果；
  特别区分服务端执行超时和客户端等待超时，避免排障时把通信故障错判成机器人执行失败。

为什么这样设计：

- topic 适合广播状态和瞬时事件，例如 ASR final、动作候选、监控日志。
- ROS 2 Action 适合“移动一秒”“转九十度”这种有持续时间、可取消、需要反馈的动作。
- 自定义 msg/action 让动作接口可测试、可限幅、可扩展，比纯字符串事件载荷更工程化。
- Lifecycle 让“进程存在”和“节点允许接单”成为两个状态：inactive 明确拒绝命令，cleanup
  释放 client/publisher/timer，避免重启整个进程才能恢复。

方案对比：

- 只用 `/cmd_vel`：简单，但 LLM 直接控制速度风险高，也难以表达执行结果。
- 只用 service：适合短请求，不适合持续动作和取消。
- 只用字符串事件 topic：开发快，但类型不安全，后期维护和测试成本高。
- 保留一个独立 demo client：比 bridge 更适合讲解 rclcpp_action 的 goal/feedback/result
  生命周期，也能在没有 Agent 的情况下单独验证 action server。结构化审计比日志关键字
  `grep` 更可靠，可供 CI 或发布门禁消费。
- 拆成三个新 ROS package 会让 launch、依赖和发布矩阵膨胀；本项目选择同包多 target，既隔离
  编译/链接依赖，又保持现有 executable、topic、参数和 action 接口兼容。

### 1.1 运行状态中间件：为什么状态 topic 也要强类型

关键代码：

- `src/embodied_agent_interfaces/msg/AudioFrontendStatus.msg`
- `src/embodied_agent_interfaces/msg/VadEvent.msg`
- `src/embodied_agent_interfaces/msg/KwsEvent.msg`
- `src/embodied_agent_interfaces/msg/KwsScore.msg`
- `src/embodied_agent_interfaces/msg/SimulationState.msg`
- `src/embodied_agent_interfaces/msg/RobotActionAck.msg`
- `src/embodied_agent_interfaces/msg/BehaviorTreeStatus.msg`
- `src/embodied_agent_core/embodied_agent_core/runtime_status_transport.py`
- `src/embodied_agent_cpp/src/audio_frontend_node.cpp`
- `src/embodied_simulation/src/simulation_control_node.cpp`

设计方式：

- `embodied_agent_interfaces` 是跨进程契约的唯一来源；Python/C++ 发布者不再自行拼 JSON。
- `runtime_status_transport.py` 只负责“领域数据/ROS 消息/可读报告字典”的边界转换，monitor
  和测试可以继续输出易读 JSON，但 ROS graph 内传输的是可发现、可校验的消息类型。
- 可选距离使用 `*_valid + value` 表达，避免用 JSON `null`；状态与结果使用枚举，避免
  `success/succeeded/done` 等自由字符串漂移。
- 音频 PCM 保持 best-effort，命令与状态按语义选 QoS；高频数据和可靠控制面不混用同一策略。

为什么这样设计：

- ROS 2 在 discovery 阶段就能发现同 topic 类型冲突，编译器和 rosidl 还能约束字段；JSON
  字符串只能等运行时解析后才暴露拼写、缺字段和类型错误。
- 多语言系统中，消息定义比散落在 Python/C++ 中的字典约定更适合作为团队接口文档。
- 报告序列化与实时中间件职责分离后，测试证据仍可保存为 JSON/JSONL，同时不会让文件格式
  反向污染实时控制接口。

方案对比：

- `std_msgs/String + JSON`：原型快，但无 schema、重复解析、跨节点容易漂移。
- 全部改 service：状态广播和连续指标不适合请求/响应模型。
- 自定义 msg + Action：事件/状态用 msg，长动作生命周期用 Action，职责更清楚；代价是接口变更
  需要重新 build，但这正是工程化版本管理应显式承担的成本。

## 2. ActionGuard：LLM 输出和机器人执行之间的安全边界

关键代码：

- `src/embodied_agent_cpp/src/action_guard_node.cpp`
- `src/embodied_agent_cpp/include/embodied_agent_cpp/guarded_command_outbox.hpp`
- `src/embodied_agent_cpp/src/guarded_command_outbox.cpp`
- `src/embodied_agent_cpp/include/embodied_agent_cpp/action_validator.hpp`
- `src/embodied_agent_cpp/src/action_validator.cpp`
- `src/embodied_agent_core/embodied_agent_core/ros_action_transport.py`

设计方式：

- 订阅 `/agent/action_candidate`。
- 接收强类型动作候选；不再解析 ROS topic 中的 JSON 字符串。
- `/agent/command_queue` 与 `/agent/command_execution` 也分别使用
  `CommandQueueEvent`、`CommandExecutionEvent`；`ros_event_transport.py` 是领域 dataclass
  与 ROS 消息之间唯一的 Adapter，未知事件会在进入 ROS graph 前被拒绝。
- 唤醒、识别与 NLU 分别使用 `WakeEvent`、`RecognitionFeedback`、`NluParseEvent`。
  `NluParseEvent` 继续组合 `NluCommand`、`CommandSlot` 和 `RobotCommand`，既保留
  可观测的语义槽位，又不让任意字典穿过 ROS 中间件边界。
- `agent_ros_io.py` 把在线/离线节点共同的 publisher/subscription 接线收敛成 Facade；
  `ros_topics.py` 集中全部 Agent topic，节点只注入业务 callback，不再拼消息或硬编码接口名。
- `ros_qos.py` 与 C++ `qos_profiles.hpp` 使用相同的六类命名语义：command/event
  使用 reliable + volatile，state 使用 reliable + transient-local，sensor/audio 使用
  best-effort，diagnostics 使用可靠浅队列。VAD/KWS/声纹节点只能选择这些 profile，禁止
  自行拼 `QoSProfile`；这样修改一次即可同时约束在线、离线和所有语音 Adapter。
- command 与 event 都是 reliable，但不能合并概念：command 可能改变机器人状态，必须禁止
  transient-local 重放；event 描述已发生的生命周期事实。state 才允许 late joiner 获取最新值。
- C++ 的 ActionGuard、Action scheduler、音频前端、硬件 Adapter 与仿真节点复用同一套名称，
  repository guard 会同时扫描 Python 与 C++，防止后续节点重新引入魔法 `depth=10`。
- 校验动作类型、速度、时长、颜色、模式等字段。
- 通过后发布 `/robot/action_command_typed` 强类型 ROS 2 msg。
- Guard 与 scheduler 尚未完成 DDS discovery 时，命令进入有界 TTL outbox；匹配后
  FIFO 转发，超时则明确拒绝，避免启动阶段静默丢动作或晚到执行旧动作。
- 拒绝时发布 `/robot/action_rejected`。

为什么这样设计：

- 大模型输出不可完全信任，必须在进入机器人执行层前做白名单和限幅。
- 删除旧字符串动作命令入口，避免仿真/硬件执行层出现双入口。
- 使用 typed message，方便 C++、Action、仿真执行器稳定对接。
- candidate 的 `ARC` 保留上层语义，ActionGuard 校验后规范化为执行层 `MOVE`，
  兼顾报告可解释性和底层速度控制复用。
- reliable QoS 只保证已经匹配的 endpoint 之间可靠，并不回放 discovery 前的消息；
  控制命令也不适合 transient-local，因为节点重启后重放旧移动命令有安全风险。
  因此这里选择应用层短期 outbox，并用 TTL 明确限制有效窗口。

中间件方案对比：

- 所有 topic 都用 reliable/depth=10：写法简单，但 PCM/scan 容易积压，状态又无法服务晚加入监控。
- 所有状态都 transient-local：监控方便，但控制命令可能在节点重启后被重放，存在安全风险。
- 按领域语义命名 QoS：调用处能直接表达 command/event/state/sensor/audio，策略可单测并跨包复用。
- Facade 与直接在节点里创建 topic：Facade 多一个明确边界，但能保证 online/offline 接口完全
  同构；新增 topic 或调整 QoS 只改一个模块，结构测试禁止节点重新出现硬编码 topic。

### 组件健康与系统就绪

关键代码：

- `src/embodied_agent_interfaces/msg/ComponentHealth.msg`
- `src/embodied_agent_interfaces/msg/SystemReadiness.msg`
- `src/embodied_agent_middleware/include/embodied_agent_middleware/component_health_registry.hpp`
- `src/embodied_agent_middleware/src/system_readiness_node.cpp`
- `scripts/system_readiness_check.py`

设计区别：Lifecycle 表达单个受管节点的配置/激活状态，diagnostics 表达运行质量和故障细节，
SystemReadiness 则回答“当前 launch profile 的必需组件是否全部可用”。三者互补；麦克风 RMS、
Gazebo odom/scan 等数据质量探针仍单独保留，避免把“进程活着”误当成功能可用。

### Agent Lifecycle：状态必须对应真实资源

关键代码：

- `src/embodied_online_agent/embodied_online_agent/online_agent_node.py`
- `src/embodied_offline_agent/embodied_offline_agent/offline_agent_node.py`
- `src/embodied_agent_core/embodied_agent_core/agent_execution_runtime.py`
- `src/embodied_agent_core/embodied_agent_core/agent_lifecycle_runtime.py`
- `src/embodied_agent_core/embodied_agent_core/asr_endpoint_runtime.py`
- `scripts/smoke_test_agent_lifecycle.sh`
- `tests/integration/test_agent_lifecycle.py`

生命周期映射为：`configure` 创建/预热 ASR、LLM、TTS 并把 endpoint/execution 所有权
绑定给 `AgentLifecycleRuntime`；`activate` 激活 managed
publisher 并启动 ASR/命令 worker；`deactivate` 关闭输入门、取消 endpoint timer、清队列、
发布 priority STOP，再协作取消模型流和等待线程；`cleanup` 关闭连接并释放 provider。
`AgentExecutionRuntime` 统一拥有连续 worker 和非连续 turn 线程，停用时通过取消标志让 token
循环尽快退出；若线程未在超时内静默，transition 明确失败，不会悄悄启动第二套 worker。

launch manager 的顺序是 `ActionGuard → Agent`：启动时下游安全边界先就绪，停用时按逆序让
Agent 先停车、Guard 后退出。独立 `ros2 run` 使用内部 autostart 保持调试便利；组合 launch
关闭内部 autostart，防止 manager 与节点同时触发 transition。deactivate 还会在 lifecycle
publisher 关闭前发布 `ComponentHealth.STATE_STOPPED`，覆盖 transient-local 的旧 READY 缓存。

方案对比：

- 只增加 lifecycle service、资源仍在构造函数启动：状态可查询，但无法安全停用或重建。
- 每次 deactivate 都销毁模型：语义简单，但重激活延迟大。
- 当前方案：模型在 configure/cleanup 间持有，I/O 在线程在 activate/deactivate 间运行；
  兼顾资源语义、快速重激活和异常恢复。
- 共享 Lifecycle 基类：能少写 callback，但 provider 字段和虚函数容易形成脆弱父类；当前采用
  组合式 runtime，在线/离线节点注入 start/stop hook，安全状态机可脱离 ROS 单元测试。

### 参数 schema 与 launch profile：为什么配置也是接口

关键代码：

- `src/embodied_agent_core/embodied_agent_core/agent_parameters.py`
- `src/embodied_agent_bringup/embodied_agent_bringup/agent_launch_contract.py`
- `src/embodied_agent_bringup/embodied_agent_bringup/agent_deployment_launch_contract.py`
- `src/embodied_agent_bringup/embodied_agent_bringup/voice_frontend_launch_contract.py`
- `src/embodied_online_agent/embodied_online_agent/online_agent_node.py`
- `src/embodied_offline_agent/embodied_offline_agent/offline_agent_node.py`
- `src/embodied_online_agent/launch/online_agent.launch.py`
- `src/embodied_offline_agent/launch/offline_agent.launch.py`
- `src/embodied_simulation/launch/voice_turtlebot3.launch.py`
- `src/embodied_simulation/launch/voice_nav2_turtlebot3.launch.py`

设计方式：

- `ParameterSpec` 同时定义默认值、说明、数值范围、枚举和额外校验；公共控制面参数只写一次，
  online/offline provider 参数分别扩展。
- `declare_agent_parameters()` 在创建 ASR/LLM/TTS、队列和 worker 前声明并校验所有参数，
  返回不可变 `AgentParameters` 快照。ROS descriptor 标记为 `read_only`，防止 `ros2 param set`
  表面成功但已经创建的 provider 没有同步更新。
- 跨字段校验会检查 `asr_partial_max_age_s >= asr_commit_delay_ms / 1000`；否则 endpoint
  为等待尾部而延迟 commit 时，partial 反而先过期。
- `agent_launch_contract.py` 从同一 schema 生成 launch 默认值和带明确 ROS 类型的
  `ParameterValue`，并为 Gazebo/Nav2 生成相同的 include 转发表。
- `voice_frontend_launch_contract.py` 进一步把 audio frontend、Silero/WebRTC VAD、KWS、
  speaker identity 的参数声明和 Node 构造收进一个深模块。在线/离线 launch 从数百行
  重复装配缩减为 provider、TTS、Lifecycle 和硬件拓扑说明。
- `agent_deployment_launch_contract.py` 把 ActionGuard、唯一 Lifecycle manager 和硬件
  Adapter 视为一个安全部署单元；`node_names=[action_guard, agent]` 同时表达正序激活和
  逆序停机，避免两份 launch 的顺序在维护中漂移。
- 配置优先级是“节点 schema → provider YAML → launch 覆盖”。YAML 只保留模型 endpoint、
  路径、线程数等 provider 配置；会话/队列/记忆默认值不再复制。

为什么这样设计：

- 参数名和默认值本质上是部署接口。三处手写会造成在线能启动、离线才在运行中报错，或修改
  YAML 后被 launch 的旧默认值悄悄覆盖。
- 启动时 fail-fast 比执行第一条语音后才发现队列容量为 0、TTS provider 拼错更容易排障，
  也避免部分节点已经 ready 后系统才退化。
- 上层仿真只透传现场经常调整的体验参数；模型细节留在 provider profile，使 launch 保持
  “编排进程拓扑”的职责，而不是变成几百行万能参数总线。
- 共享 contract 只接收 `config` 和 `capture_default`，没有读取在线/离线 provider 对象；
  这条窄接口避免共享模块反向依赖具体 Agent 包。

方案对比：

- 每个节点内 `declare_parameter(name, default)`：局部直观，但 online/offline 和 YAML 很容易漂移。
- 只依赖 YAML：部署灵活，却缺少范围/枚举和跨字段校验，也难以给 `ros2 param describe` 提供元数据。
- 动态参数回调热更新：适合 PID 或阈值等真正支持重配置的组件；Agent 的 provider、线程和队列
  在构造期绑定，完整热更新需要事务式重建，因此当前选择只读快照和受控重启更可靠。
- 共享 schema + 薄 launch 契约：多一个抽象模块，但默认值、类型、校验和对外参数面都可单测，
  更适合在线/离线两条实现长期并行维护。

ActionGuard 方案对比：

- 在 prompt 里约束模型：必要但不够，模型仍可能输出非法字段。
- 在执行器里校验：太晚，安全边界分散。
- 单独 ActionGuard：边界清晰，便于单测和面试讲解。

## 3. 连续语音会话：一次唤醒，多轮控制

关键代码：

- `src/embodied_agent_core/embodied_agent_core/continuous_voice.py`
- `src/embodied_agent_core/embodied_agent_core/agent_execution_runtime.py`
- `src/embodied_agent_core/embodied_agent_core/agent_control_plane.py`
- `src/embodied_agent_core/embodied_agent_core/agent_control_plane.py`
- `src/embodied_agent_core/embodied_agent_core/ros_agent_events.py`
- `src/embodied_agent_core/embodied_agent_core/wakeword.py`
- `src/embodied_agent_core/embodied_agent_core/wake_provider.py`
- `scripts/continuous_voice_monitor.py`

设计方式：

- `ContinuousVoiceSession` 负责判断一句 ASR final 是唤醒、命令、拒绝还是休眠。
- `AgentControlPlane.accept_transcript()` 在 session 之上统一归一化、短命令补全、
  retry、session sleep、急停/取消导航和清队列决策；online/offline 不再复制该流程。
- `RosAgentEventPublisher.publish_control_decision()` 把领域决策适配成 typed ROS 事件，
  让领域核心可以脱离 ROS graph 做单元测试。
- 支持唤醒词别名，例如“小志”“晓智”。
- 支持 filler 过滤，例如“嗯”“啊”。
- 支持 duplicate window，过滤短时间重复 ASR final。
- 支持 session timeout，超时后必须重新唤醒。

为什么这样设计：

- 真实麦克风会持续产生 ASR final，如果每句话都直接进 LLM，会出现误触发和卡顿。
- 会话层把“听到了什么”和“是否应该执行”分开，便于监控和调参。
- 使用组合而不是让两个节点继承大型基类：控制面拥有状态机，节点拥有 provider，
  ROS Adapter 只负责传输，三个变化方向可以独立测试和替换。
- 文本唤醒先跑通，不强依赖声学 KWS 模型，部署更稳。

方案对比：

- 每条命令都要求带“小智”：安全但体验差。
- 完全不用唤醒：误触发多，不适合现场演示。
- 声学 KWS 优先：体验更好，但依赖模型和音频环境；当前项目保留 seam，默认用文本唤醒保证可部署。

## 4. 连续命令队列与急停抢占

关键代码：

- `src/embodied_agent_core/embodied_agent_core/continuous_voice.py`
- `src/embodied_online_agent/embodied_online_agent/online_agent_node.py`
- `src/embodied_offline_agent/embodied_offline_agent/offline_agent_node.py`
- `src/embodied_agent_core/embodied_agent_core/agent_application_runtime.py`

设计方式：

- 普通命令进入 `ContinuousCommandQueue`，按 FIFO 顺序执行。
- `AgentControlPlane.enqueue_command()` 统一 NLU 拆批、batch metadata、retry 和 queue_full；
  离线链路只通过私有 context 附加 latency，不得覆盖公共 batch 字段。
- `AgentApplicationRuntime.run_queued_turn()` 恢复入队时冻结的用户/延迟上下文；
  `AgentExecutionRuntime` 的 worker 只负责线程所有权、busy 和异常隔离。
- 命令执行前发布 started，执行后发布 finished。
- worker 用 `finally` 统一复位 busy；单条命令异常发布 `success=false` 后继续消费下一条，
  避免 3～5 分钟演示被一次 TTS/LLM 异常永久终止。
- `停下/急停` 是 priority stop：清空队列、取消当前 sequence、立即发布 stop。
- 非优先命令设置 TTL，太旧会过期丢弃并上报。

控制层还有第二级 C++ 队列，关键代码为 `action_scheduler.hpp/.cpp`：Python 队列管理“哪句用户命令先处理”，C++ 队列管理“哪个受信 Action goal 先执行”。组合动作会先批量发布给 C++；`RobotCommand.priority` 明确区分用户急停和计划 STOP。这样旁路发布者也不能绕过 FIFO，且 Action Client、取消 watchdog、错误码和 `/diagnostics` 都集中在同一个模块。

为什么这样设计：

- 用户会连续说多条命令，不能因为上一条动作 busy 就静默丢弃下一条。
- 急停不能排队等待，必须抢占。
- TTL 防止机器人执行用户很久之前说过、已经过时的命令。

方案对比：

- busy 时直接丢弃：实现简单，但体验像“卡住”。
- 并行执行所有命令：机器人动作冲突，安全性差。
- FIFO + priority stop：兼顾连续体验和安全边界。

## 4.1 LLM 流式协议为什么需要独立运行时

关键代码：

- `src/embodied_agent_core/embodied_agent_core/streaming_turn.py`
- `src/embodied_agent_core/embodied_agent_core/protocol.py`
- `src/embodied_agent_core/embodied_agent_core/command_fallback.py`
- `src/embodied_online_agent/embodied_online_agent/online_turn_runtime.py`
- `src/embodied_offline_agent/embodied_offline_agent/offline_turn_runtime.py`

`StreamingTurnRuntime` 隐藏 `TaggedStreamParser`、`SentenceChunker`、确定性命令优先级和
语义安全拦截。`OnlineStreamingTurnRuntime` 把 `on_speakable` 接到网络 TTS 队列，
`OfflineStreamingTurnRuntime` 接到伪流式双缓冲，因此复用的是“稳定协议”，不是强行
复用不同 provider 的音频实现；两个 ROS 节点只负责装配这些 Adapter。

动作选择顺序固定为：明确中文命令的 deterministic parser → 语义安全阻断 → 模型动作。
完成后返回不可变 `StreamingTurnResult`，记忆、动作发布和日志都使用同一份已选择结果，
避免出现“动作被安全层拦截，但用户画像却记录为成功执行”的分裂状态。

方案对比：

- 两个节点复制 token loop：短期直观，但协议修复和安全策略容易只改一边。
- 继承大型 `BaseAgentNode`：可减少代码，却会把 ROS、provider、TTS 和 latency 耦合到一起。
- 深模块 + callback adapter：公共规则只有一份，在线/离线只保留真正不同的 I/O 和指标。

## 5. VAD、endpoint 与 ASR commit delay

关键代码：

- `src/embodied_agent_cpp/src/audio_frontend_node.cpp`
- `src/embodied_agent_cpp/src/audio_processing.cpp`
- `src/embodied_online_agent/embodied_online_agent/online_agent_node.py`
- `src/embodied_offline_agent/embodied_offline_agent/offline_agent_node.py`
- `src/embodied_agent_core/embodied_agent_core/asr_endpoint_runtime.py`
- `scripts/audio_frontend_calibration.py`
- `scripts/voice_calibration_report.py`

设计方式：

- C++ audio frontend 发布 `/audio/clean_pcm`、`/audio/speech_started`、`/audio/speech_ended`、`/audio/silence_timeout`。
- `VAD_PROVIDER=auto` 会在启动脚本里先跑 `voice_provider_preflight.py`：Silero VAD 依赖可用时，
  AudioFrontend 只发布 clean PCM，`silero_vad` sidecar 负责 endpoint；Silero 不可用但
  `webrtcvad` 可用时，`webrtc_vad` sidecar 接管 endpoint；都不可用时降级 energy VAD。
- `scripts/setup_voice_vad_runtime.sh` 提供 WebRTC/Silero 可选依赖安装入口，支持 dry-run；
- Silero 默认使用项目内 `SileroOnnxVadProvider` 管理模型的递归 `state` 和 64-sample
  context，16kHz 每 512 samples（32ms）推理一次。相比官方 PyTorch 包路径，端侧默认只需
  约 2.2MiB ONNX 模型和 ONNX Runtime；模型版本、哈希与实测延迟由验收报告记录。
- `StreamingVadEndpoint` 使用“连续帧起点确认 + 较低结束阈值”的状态机：起点去抖负责过滤
  短噪声，阈值滞回负责避免概率在临界值附近反复切换。相比单一能量阈值，它更适合长时间
  麦克风控制；相比直接调用模型工具函数，独立 endpoint 状态机更容易单测和替换 provider。
  它会安装 `embodied_voice_frontend[webrtc-vad]`、`embodied_voice_frontend[silero-vad]`
  对应 extra，并在安装后跑 provider preflight。
- `voice_provider_preflight.py` 不只判断 PASS/BLOCKED，还会在 auto 降级或显式 provider
  缺依赖时输出 `recommendations`。这样真实麦克风演示前可以从“缺什么包”直接走到
  “运行哪个 setup 脚本”，减少现场排障成本。
- `webrtc-vad-sidecar` 是安装 WebRTC runtime 后的显式验收入口：它启动 C++ audio frontend
  和 `webrtc_vad` sidecar，确认端点事件由成熟 VAD 接管，而不只是检查 Python 包是否存在。
- `silero-vad-runtime` 是更强的 Silero 验收：先使用真实语音测模型概率/单帧延迟，再向
  ROS 2 sidecar 发布 PCM，要求输出成对 endpoint 事件。
- `scripts/setup_voice_kws_runtime.sh` 提供 openWakeWord、sherpa-onnx KWS、LiveKit WakeWord
  的可选运行时入口；sherpa profile 会复用 ZipFormer ASR 模型路径，生成默认关键词文件，
  并写出 `logs/sherpa_kws.env`，方便后续 `source` 后直接跑 `provider-preflight`。
- `sherpa-kws-sidecar` 会实际启动 `sherpa_onnx.KeywordSpotter`，证明声学 KWS 不只是
  参数 seam；默认关键词文件使用 `小 智` / `你 好 小 智` 这种 tokenized 写法，
  避免 sherpa 无法从 tokens.txt 编码整句中文。
- Agent 收到 endpoint 后交给 `AsrEndpointRuntime`：50ms 内重复端点只接受一次，延迟 timer
  在节点关闭时统一取消，且非连续模式 busy 时不会误提交下一句。
- runtime 只依赖 callback；在线 callback 直接调用 ASR commit，离线 callback 把 commit
  放入音频处理队列，因此并发策略一致而 provider 传输方式保持独立。
- `asr_commit_delay_ms` 允许在 endpoint 后等待少量时间，再提交 final。
- `VOICE_CONTROL_PROFILE` 提供 normal、quiet、low_gain、noisy_room 四种参数预设。
- `voice_calibration_report.py` 把 provider preflight、audio calibration、KWS score calibration
  汇总成 `logs/voice_calibration_report.json/.md`，并额外生成可 `source` 的
  `logs/voice_calibration.env`，用于真实麦克风演示前保存和复用调参证据。对
  openWakeWord/LiveKit，采到 KWS 分数后会把建议阈值写成
  `OPENWAKEWORD_THRESHOLD` / `LIVEKIT_WAKEWORD_THRESHOLD`。
- `continuous_voice_control.sh` 默认 `APPLY_VOICE_CALIBRATION=auto`：如果
  `logs/voice_calibration.env` 存在，会在 profile 默认值计算前加载；如果用户显式传入
  `VOICE_CONTROL_PROFILE/SPEECH_START_THRESHOLD/VAD_PROVIDER` 等关键变量，则显式值优先。
- `transcript_stabilizer.py` 保存同一 utterance 的最新 ASR partial；final 严格截短且尾部是
  可解释控制槽位时才恢复，并通过 `asr_final_recovered` feedback 留证。

为什么这样设计：

- 真实 ASR 容易漏掉尾部数字和量词，例如“左转90度”只 final 成“左转”。
- 适当延迟 300～500ms 可以换取更完整的识别结果。
- 校准文件默认自动复用，可以减少演示前忘记 `source logs/voice_calibration.env` 的概率；
  但显式环境变量优先，避免旧校准文件覆盖现场临时调参。
- 把 VAD 依赖安装封装成项目脚本，是为了让“成熟 VAD sidecar”不只是代码 seam；
  演示环境可以通过 dry-run、install、preflight 三步确认真的没有降级到 energy VAD。
- KWS 单独做 runtime setup，是因为“唤醒词检测”比文本触发更接近真实机器人交互；openWakeWord
  适合快速准备 Python runtime，sherpa KWS 则能复用离线 ASR 运行时资产。
- VAD 和 commit 分离，便于定位“音频没听到”和“ASR final 太早”两类问题。
- partial/final 合并放在 Agent 入口而不是 ASR provider 内，在线 Qwen 与离线 ZipFormer
  共用同一安全策略，provider 仍只负责忠实上报模型结果。
- 成熟 VAD 做成 sidecar，而不是塞进 PortAudio 回调线程，是为了避免模型推理阻塞音频采集。

方案对比：

- 极短静音阈值：响应快，但尾部漏识别多。
- 很长静音阈值：完整但交互迟钝。
- profile + commit delay：保留可调空间，适合不同环境。
- auto Silero/WebRTC sidecar：Silero 判断更稳但依赖较重，WebRTC VAD 更轻、更易部署但只有二分类；
  降级 energy VAD 保证基础演示不被可选依赖卡死。
- 直接采用最长 partial：召回高但可能恢复模型中途幻觉；当前实现要求 final 前缀关系、2 秒新鲜度和
  安全槽位白名单，牺牲部分召回换取动作安全。

## 6. 短命令补全与模糊归一化

关键代码：

- `src/embodied_agent_core/embodied_agent_core/command_normalizer.py`
- `src/embodied_agent_core/config/command_normalization_zh.yaml`
- `src/embodied_agent_core/embodied_agent_core/command_completion.py`
- `src/embodied_agent_core/embodied_agent_core/transcript_stabilizer.py`
- `src/embodied_agent_core/test/test_command_completion.py`

设计方式：

- command normalizer 处理错别字、同音词、常见 ASR 误识别。
- command completer 处理缺槽短命令：
  - `前进` → `前进一秒`
  - `后退` → `后退一秒`
  - `左转` → `左转九十度`
  - `右转` → `右转九十度`
- 补全只作用于普通控制命令，不处理 `停下/急停`。

为什么这样设计：

- 演示中“短 ASR final”不应该让链路中断。
- 用规则补全比再调一次 LLM 更快、更稳定、可测试。
- 安全命令保持原样，避免误改写。

方案对比：

- 全交给 LLM：泛化强，但慢且不可预测。
- 全靠 ASR 热词：能改善识别，但不能解决所有尾部漏识别。
- 规则补全 + ASR 调参：更适合当前演示目标。

## 7. 在线 Agent：流式 ASR/LLM/TTS 与动作回调

关键代码：

- `src/embodied_online_agent/embodied_online_agent/online_agent_node.py`
- `src/embodied_online_agent/embodied_online_agent/providers/qwen_asr.py`
- `src/embodied_online_agent/embodied_online_agent/providers/openai_compatible_llm.py`
- `src/embodied_online_agent/embodied_online_agent/providers/qwen_tts.py`
- `src/embodied_agent_core/prompts/system_prompt_zh.txt`
- `src/embodied_agent_core/embodied_agent_core/protocol.py`

设计方式：

- ASR partial/final 分开发布。
- LLM 输出通过 tagged stream parser 解析文本和动作。
- TTS 按句子 chunk 合成，减少首包等待。
- 动作候选通过 `/agent/action_candidate` 进入 ActionGuard。
- metrics 记录 LLM 首 token、TTS 首音频等延迟。

为什么这样设计：

- 流式交互要尽早给用户反馈，不能等完整回复生成完。
- 动作回调必须结构化，不能从自然语言里临时猜。
- TTS 与 LLM 分块并行，降低体感延迟。

方案对比：

- 非流式 LLM/TTS：实现简单，但等待时间长。
- LLM 直接发控制 topic：快但安全边界差。
- Agent 只产动作候选，C++ guard 再执行：更符合机器人系统分层。

## 8. 轻量 NLU：一句话多个命令识别

关键代码：

- `src/embodied_agent_core/embodied_agent_core/command_nlu.py`
- `src/embodied_agent_core/config/command_nlu_zh.json`
- `scripts/train_command_nlu.py`
- `tests/integration/test_continuous_multi_command.py`

设计方式：

- 使用字符 n-gram 原型模型识别控制意图，不依赖 torch/transformers。
- 输入一条 ASR final，输出多个动作片段和置信度。
- `ParsedCommand.slots` 显式保存方向、速度、距离、角度、时长或地点，不再只从最终
  `linear_x/angular_z/duration_s` 反推动作语义。
- 距离动作按 `duration = distance / speed` 转换到现有强类型运动接口；未指定速度且超过
  默认 10 秒窗口时，在 `0.5m/s` 安全上限内自适应提速。用户已明确慢速且单动作无法在
  10 秒内完成时，拆成多个同速 primitive action 顺序执行，避免提速或静默截断距离。
- 任意角度先转换为弧度，再根据角速度计算持续时间；90° 保留已有 Gazebo 标定参数，
  360° 自动使用安全范围内更高的角速度，避免突破 ActionGuard 的时长上限。
- 例如“向右转，向前走一秒”会输出 `turn -> move`。
- 每个队列项带 `batch_id / batch_index / batch_size`，便于 monitor 解释顺序。
- 每个动作候选带 `request_id`，ActionGuard 映射成 `RobotCommand.command_id`，用于 result 关联。
- monitor 支持 `CONTINUOUS_SAMPLE_LOG=logs/asr_nlu_samples.jsonl`，把真实 ASR final、
  NLU/补全/归一化 feedback、动作候选和 result 写成 JSONL，方便把现场错词沉淀成回归集。
- `scripts/asr_nlu_samples_to_eval_candidates.py` 会把这些运行时事件按 `asr_final`
  分组，生成 `logs/asr_nlu_eval_candidates.jsonl`；候选样本带 `suggested_eval_case`，
  人工确认后即可补进 `training/robot_instruction_eval.jsonl`。
- `scripts/evaluate_asr_nlu_eval_candidates.py` 复用正式 parser 评估逻辑，对候选集先跑
  临时 accuracy；这让现场采到的错词即使还没合入正式数据集，也能马上用于回归观察。

为什么这样设计：

- 纯字符串 split 对无标点语音不稳，例如“向右转向前走一秒”。
- 大模型理解更强，但慢、不可预测、在线成本高。
- 轻量 NLU 覆盖固定机器人动作域，速度快、可测试、可解释。
- 真实 ASR 的错词分布很依赖麦克风和环境，靠人工凭记忆补测试很容易漏；采样日志转候选集
  可以把现场失败直接变成可回归的数据资产。
- 候选集和正式评估集分开，是为了避免“观察到的动作候选”未经人工确认就污染 ground truth；
  但候选集临时评估又能让工程迭代保持速度。

方案对比：

- 规则拆分：部署最简单，但表达能力弱。
- 大模型 function calling：泛化强，但响应和稳定性受模型影响。
- 本地轻量 NLU + ActionGuard：在固定动作域内更适合端侧演示。

槽位评测：

```bash
bash scripts/acceptance_test.sh instruction-parser-eval
```

当前 `training/robot_instruction_eval.jsonl` 包含 43 条代表用例；报告按 `slots / speed /
distance / angle / duration / navigation` 等 tag 分组，避免只用总体准确率掩盖某一类槽位失败。

## 9. 声纹识别与用户行为记忆

关键代码：

- `src/embodied_voice_frontend/embodied_voice_frontend/speaker_identity_node.py`
- `src/embodied_agent_core/embodied_agent_core/memory_command_service.py`
- `src/embodied_agent_core/embodied_agent_core/user_context_runtime.py`
- `src/embodied_agent_core/embodied_agent_core/speaker_transport.py`
- `src/embodied_agent_interfaces/msg/SpeakerIdentity.msg`
- `src/embodied_agent_interfaces/msg/SpeakerEnrollRequest.msg`
- `src/embodied_agent_interfaces/msg/SpeakerEnrollStatus.msg`
- `src/embodied_agent_core/embodied_agent_core/user_memory.py`
- `src/embodied_agent_core/embodied_agent_core/user_preferences.py`
- `src/embodied_online_agent/embodied_online_agent/online_agent_node.py`
- `src/embodied_offline_agent/embodied_offline_agent/offline_agent_node.py`
- `tests/integration/test_speaker_memory_mock.py`
- `tests/integration/test_sherpa_speaker_identity_ros.py`
- `scripts/setup_sherpa_speaker_runtime.sh`
- `scripts/probe_sherpa_speaker_runtime.py`

设计方式：

- 声纹识别被做成 sidecar：订阅 `/audio/clean_pcm` 和 `/audio/speech_ended`，发布 typed `/agent/speaker_identity`。
- 声纹录入通过 typed `/agent/speaker_enroll_request` 触发，sidecar 把后续语音段保存成 wav 样本并维护 `speakers.txt`，进度通过 `SpeakerEnrollStatus` 枚举发布。
- `MemoryCommandService.handle()` 是在线/离线共用的深模块接口：查询身份、保存/删除偏好、清空记忆、文本兜底录入和 interaction 记录都隐藏在实现内部；节点只处理 ROS 发布和 TTS。
- `UserContextRuntime` 统一持有当前身份和 `MemoryCommandService`。每条命令入队时冻结
  `UserContextSnapshot`，prompt 摘要、动作偏好和最终 interaction 都使用这份快照；这解决了
  长动作期间声纹从 A 更新成 B 后把 A 的命令误写入 B 画像的并发竞态。
- `speaker_transport.py` 是声纹领域对象与 rosidl 消息之间唯一 Adapter，置信度门槛在进入领域层时统一应用。
- Sherpa backend 按 speaker 聚合多段 embedding 后一次注册到
  `SpeakerEmbeddingManager`，确保注册时采集的 3 段样本都参与模板，而非只保留第一段。
- 匹配时遍历 `all_speakers` 获取真实 `score`，发布 top-1 confidence、第二名分数和
  score margin；低于阈值或 top-1/top-2 过近均返回 `unknown`。
- Sherpa 模式启动时只发布 `awaiting_audio/unknown`，不会用 demo mock 身份抢先加载个人记忆。
- Agent 只消费稳定 typed `SpeakerIdentity`，不直接绑定某个模型库。
- `UserMemoryStore` 按 `speaker_id` 保存本地 profile，包括用户名、偏好、常用动作、最近交互。
- Agent 推理前把当前用户画像追加进 system prompt，但动作仍必须经过 ActionGuard。
- `user_preferences.py` 在动作发布出口统一应用确定性偏好，例如 `movement_speed=slow/fast`、
  `default_move_duration_s`、`default_turn_degrees`；这样 fallback、轻量 NLU、多命令队列和 LLM 输出
  都能得到一致的参数调整。
- 管理命令直接在 Agent 层处理，例如“记住我，我是小李”“我喜欢慢一点”“我是谁”“清除我的记忆”。
- 记忆生命周期支持“我的偏好”查询、“恢复默认速度”等按项删除，以及清空整个 profile。
  清空命令不会再被当作新 interaction 写回，避免出现“刚清除又生成文件”的反直觉行为。
- `user_memory_retention_days` 对 recent interaction/correction 做 TTL 清理；显式 preference
  代表用户配置，只有主动删除才消失。这样兼顾隐私留存上限与机器人行为的可预测性。

为什么这样设计：

- 声纹模型属于可替换能力，和 ASR/LLM/动作控制主链路解耦，降低演示风险。
- 用户画像是长期稳定信息，不适合无限追加到普通对话历史里。
- 原始话术明细和稳定偏好采用不同保留策略：前者 TTL，后者显式删除；如果给全部记忆
  使用同一个 TTL，机器人可能在用户不知情时突然恢复默认行为。
- 记忆写入必须可控，不能完全交给 LLM 自行决定，否则容易把误识别或幻觉写入本地 profile。
- 低置信度声纹返回 `unknown`；`UserMemoryStore` 对写操作增加 `LowConfidenceSpeakerError`
  门控，`UserContextRuntime` 捕获后跳过个人记忆写入，避免把 A 用户偏好误写到 B 用户。
- 偏好只改写低层运动参数，且只在 speaker identity 可信时生效；真正的速度/时长边界继续由
  C++ ActionGuard 兜底，避免“记忆”绕过安全策略。

方案对比：

- 直接接 mem0/Letta/Zep：记忆能力强，但偏 Web Agent/服务端框架，当前 ROS2 端侧项目会变重。
- 使用 SpeechBrain/pyannote：模型能力成熟，但依赖 PyTorch 或 HuggingFace 模型，部署复杂。
- 当前方案：mock 可自动验收，sherpa-onnx seam 可接真实端侧声纹，和已有离线技术栈一致。
- 当前实测：官方 3D-Speaker 中文 ONNX 已完成真实 CPU embedding 与 ROS sidecar 自匹配；
  仍未完成多人、多房间数据集上的 FAR/FRR 评估，因此不宣称“声纹准确率已达生产级”。
- 模型与 API 选型参考 Sherpa-ONNX 官方
  [Speaker Identification](https://k2-fsa.github.io/sherpa/onnx/speaker-identification/index.html)；
  setup 脚本固定官方模型 URL 与 SHA256，避免模型文件悄然变化。

验收方式：

```bash
bash scripts/acceptance_test.sh speaker-memory-mock
bash scripts/acceptance_test.sh speaker-enroll
bash scripts/setup_sherpa_speaker_runtime.sh
bash scripts/acceptance_test.sh speaker-runtime
```

这些验收分别证明：speaker identity 进入 Agent、用户偏好落盘、偏好能影响后续动作参数、
动作执行后更新用户行为统计、录入流程能采集样本，以及真实 Sherpa embedding 能经 ROS
sidecar 输出实际相似度。多人准确率仍需另建注册/查询数据集评估。

## 10. 离线 Agent：Sherpa、llama.cpp、Sherpa-TTS/SummerTTS 与双缓冲

关键代码：

- `src/embodied_offline_agent/embodied_offline_agent/offline_agent_node.py`
- `src/embodied_offline_agent/embodied_offline_agent/providers/sherpa_asr.py`
- `src/embodied_offline_agent/embodied_offline_agent/providers/llama_cpp.py`
- `src/embodied_offline_agent/embodied_offline_agent/providers/sherpa_tts.py`
- `src/embodied_offline_agent/embodied_offline_agent/providers/summer_tts.py`
- `src/embodied_offline_agent/embodied_offline_agent/providers/summer_tts_ros.py`
- `src/embodied_agent_interfaces/srv/SynthesizeSpeech.srv`
- `src/embodied_agent_cpp/src/summer_tts_service_node.cpp`
- `src/embodied_offline_agent/embodied_offline_agent/pseudo_streaming_tts.py`
- `src/embodied_offline_agent/embodied_offline_agent/double_buffer.py`
- `src/embodied_offline_agent/embodied_offline_agent/latency.py`
- `scripts/start_llama_server.sh`
- `scripts/llama_cpp_preflight.py`
- `scripts/setup_summer_tts_runtime.sh`
- `scripts/summer_tts_smoke.py`
- `scripts/smoke_test_summer_pseudo_tts.py`
- `scripts/generate_offline_showcase_report.py`
- `scripts/audit_offline_showcase_evidence.py`

设计方式：

- Sherpa ZipFormer 负责流式 ASR。
- llama.cpp server 提供 OpenAI-compatible streaming completion，`LlamaCppLlm` 只暴露 `stream(messages)`，
  让 Offline Agent 不关心底层是 llama.cpp、云 API 还是测试 fake client。
- Sherpa-TTS 是默认稳定 TTS provider；SummerTTS 是新增 C++ 独立编译 TTS provider，可通过 `tts_provider:=summer` 切换。
- `SummerTts` provider 调用 `third_party/SummerTTS/build/tts_test`，输入文本文件和 `.bin` 模型，读取 16kHz mono wav 后返回 PCM16 bytes。
- `SummerTtsServiceNode` 是常驻 C++ ROS component：节点启动时加载 `single_speaker_fast.bin`，
  对外提供 `/tts/synthesize` service，Python 侧 `SummerTtsRosClient` 通过 `tts_provider:=summer_ros` 调用。
  service 内部对短文本做 LRU 缓存，缓存 key 包含文本、speaker_id 和 length_scale，避免不同音色或语速误复用。
- `PseudoStreamingTtsPipeline` 把 Sherpa/SummerTTS 这种“整句生成”的本地 TTS 包装成伪流式：
  LLM 文字增量先经 `SentenceChunker` 切成短句，TTS worker 合成 PCM，audio worker 再按小块发布。
- 双缓冲把 LLM 文本生成、TTS 合成与音频输出解耦。
- latency 模块记录离线端到端耗时。
- llama.cpp provider 请求流式 usage，分别记录 `stream_chunks`、`prompt_tokens`、
  `completion_tokens`、首 token 和 decode 估算；SSE chunk 数不再冒充 token 数。
- `runtime_warmup_enabled` 在节点 ready 前预热真实 `system + 已加载短期历史` 前缀，并预合成
  一个短 TTS；ConversationMemory 保存模型原始 `<speech>/<action>` 协议输出，使下一轮 chat
  template 与 server slot 中的生成 token 保持一致。离线短期历史限制为 3 轮，长期偏好由用户画像保存。
- llama-server 默认 `parallel=1`，让单用户语音 Agent 复用 KV cache。2026-07-11 本机实测：
  冷 prefill 约 4s（节点 ready 前承担），warm Agent turn 首 token 中位数约 `536ms`、P95
  约 `560ms`，API decode 估算中位数约 `28.9 tokens/s`。
- TTS pipeline 额外记录 `text_chunks`、`synth_calls`、`audio_chunks`、`first_text_to_first_audio_ms`，
  由 `metrics_transport.py` 映射进统一 `/agent/metrics` 的 `AgentTurnMetrics` 字段。
- 在线/离线不再维护两条 `String + JSON` 指标 topic；`AgentTurnMetrics.source` 区分来源，NaN
  表达缺失浮点值，`TARGET_UNKNOWN/MET/MISSED` 避免布尔默认值把“未采集”误写成“不达标”。
- `llama_cpp_preflight.py` 把 binary、模型文件、`/health`、`/v1/models`、低 token 流式 chat 分层验证。
- `summer_tts_smoke.py` 把 SummerTTS 源码、二进制、模型和真实合成分层验证；`summer-pseudo-tts`
  再验证真实 SummerTTS 能接入项目双缓冲伪流式 pipeline。
- `generate_offline_showcase_report.py` 汇总模型资产、运行时版本、parser accuracy、可选 latency/ASR/TTS/
  真实 Agent E2E benchmark；`offline_latency_targets.py` 明确把非流式 `synthesize()` 计为整句合成耗时，
  真正首音频只采用 `PseudoStreamingTtsPipeline` 发布第一块 PCM 的时间，避免指标命名失真；
  `audit_offline_showcase_evidence.py` 再审计这份报告，输出 `claim_guidance`，明确哪些指标已有证据、
  哪些只能作为后续计划。

为什么这样设计：

- 端侧算力有限，离线链路必须控制模型体积和串行等待。
- llama.cpp、Sherpa、SummerTTS 都是轻量本地部署方案，适合 CPU/边缘端演示。
- 双缓冲可以减少“LLM 等 TTS / TTS 等 LLM”的卡顿。
- 本地 TTS 通常不是天然流式；伪流式的关键是尽早切短句、尽早开始合成、音频按 PCM 小块发布。
- 推理层独立预检可以快速判断问题在模型服务、ASR、TTS 还是 ROS 控制链路，避免完整 demo 失败时只能猜。
- 离线展示最怕“工程接口接了”和“指标已复现”混在一起讲；证据审计脚本把未运行的 latency、
  ASR/TTS benchmark 标成 warning，并用数据集、提示词、adapter、GGUF 和评估报告 SHA256
  绑定 LoRA 证据，帮助汇报时守住边界。
- Qwen3 thinking parser 可能造成“动作 JSON 正确但 speech 起始标签缺失”。因此评估永久拆成
  `action_score`、`protocol_score`、严格 `model_score` 和 fallback `effective_score`，避免把模板
  兼容问题误判为动作语义，也避免用工程兜底冒充模型能力。
- `llama-bench` 的纯 decode 与 Agent API 长 prompt 指标必须分开：前者证明模型/CPU 上限，
  后者包含 prompt cache、历史滑窗和服务协议开销。本机同轮报告分别约为 `37.2` 与
  `28.9 tokens/s`，不能选择更高数字冒充端到端吞吐。
- SummerTTS 是 C++ 项目，适合展示“端侧 C++ 运行时嵌入”；命令行 provider 保证部署简单，
  常驻 ROS component 则展示了更工程化的低耦合封装，并减少每句进程启动和模型加载开销。
- 短文本缓存只覆盖“收到/好的/正在执行”等反馈语，长句不缓存，避免内存被长音频占满；这属于工程优化，
  不是模型推理加速，因此文档仍把 Sherpa-TTS 作为默认低延迟 gate。
- 请求失败后只在“尚未吐出 token”时重试；如果流式回复已经输出一半，就不能静默重试，否则上游 parser 会收到拼接污染的回复。

方案对比：

- 全部云端：效果强，但不体现端侧部署能力。
- Python 大模型框架直接推理：开发方便，但部署和性能压力更大。
- llama.cpp + Sherpa：工程味更强，适合展示端侧推理思路。
- SummerTTS 命令行封装：接入最快、易验收，但每句会启动进程并加载模型；适合先打通链路。
- SummerTTS C++ 组件化封装：可复用已加载模型，服务接口清晰，但需要维护 C++ wrapper、ROS2 service
  和组件生命周期；短文本缓存能显著优化重复反馈，未命中时仍受 SummerTTS CPU infer 本身限制。
- 直接绑定 llama.cpp C API：可控性更强，但 Python/ROS2 集成和维护成本高；本项目选择 OpenAI-compatible server，
  用网络 seam 换取更低耦合、更容易 mock 和更清晰的部署边界。

## 11. BehaviorTree.CPP 与 pluginlib 仿真执行

关键代码：

- `src/embodied_simulation/config/command_tree.xml`
- `src/embodied_simulation/include/embodied_simulation/active_action_runtime.hpp`
- `src/embodied_simulation/src/active_action_runtime.cpp`
- `src/embodied_simulation/include/embodied_simulation/simulation_ros_io.hpp`
- `src/embodied_simulation/src/simulation_ros_io.cpp`
- `src/embodied_simulation/src/command_behavior_tree.cpp`
- `src/embodied_simulation/include/embodied_simulation/robot_executor.hpp`
- `src/embodied_simulation/src/gazebo_robot_executor.cpp`
- `src/embodied_simulation/src/mock_robot_executor.cpp`
- `src/embodied_simulation/src/nav2_robot_executor.cpp`
- `src/embodied_simulation/src/simulation_controller.cpp`

设计方式：

- BehaviorTree 负责动作执行流程：校验动作、检查安全、执行、确认结果。
- `ActiveActionRuntime` 把“本地计时 + Nav2 外部 result + cancel/timeout + BT
  outcome”合并为一个 `ActiveActionDecision`，ROS 节点不再维护平行状态字段。
- `SimulationRosIo` 统一管理 7 个 Lifecycle publisher 及其激活/停用顺序，并把领域状态
  映射为强类型 ACK、BT status、diagnostics 和 component health；控制节点不再感知消息序号、
  BT 去重签名或各 topic 的 QoS 细节。
- `RobotExecutor` 是统一接口。
- pluginlib 提供 `GazeboRobotExecutor` 和 `MockRobotExecutor` 两种后端。
- `SimulationController` 负责把动作转换成 `/cmd_vel`，并处理基础安全逻辑。

为什么这样设计：

- BT 把流程从 if/else 里抽出来，更接近 Nav2 的工程风格。
- pluginlib 让 mock 和 Gazebo 后端可替换，测试不必依赖 Gazebo。
- executor 分层后，未来接真实硬件或 Nav2 行为树更自然。
- 三类 executor 分开编译后，Gazebo/Mock 不再携带 Nav2 Action、地图加载和线程依赖；
  新增后端只需实现 `RobotExecutor` 并注册 pluginlib，不必修改既有后端源码。
- 运行时是无 ROS Node 依赖的 C++ 深模块，可以用确定的时间值测试边界条件，
  避免用 launch 测试才能覆盖超时、取消和旧 result 等状态组合。
- ROS I/O 集中后，新增 topic 或修改 DDS 策略只有一个改动点；Lifecycle 停用时先发布 STOPPED、
  再停用 publisher，避免 transient-local 缓存仍向晚加入观察者显示 READY。

方案对比：

- 单个节点写死所有逻辑：短期快，但难测试、难扩展。
- 仅把代码机械拆成多个 helper：文件变短但状态仍散落；本项目按“一个 goal
  的完整生命周期”划分模块边界，让调用方只处理 `start/update/reset`。
- 每个 publisher 都留在 Node：直观但 QoS、激活状态、状态码映射会散落；本项目用
  `SimulationRosIo` 封装完整中间件语义，Node 只调用业务含义明确的 publish 方法。
- 直接引入完整 Nav2：功能强，但本项目目标不是复杂导航，成本过高。
- 轻量 BT + pluginlib：足够展示工程规范，同时保持项目可跑通。

### 真实语音部署 profile

- `scripts/voice_control_profile.sh` 是 `normal/quiet/low_gain/noisy_room` 的唯一默认值解析器。
- `continuous_voice_control.sh` 选择 `control` 场景，Nav2 入口选择 `navigation` 场景；后者只延长
  会话和命令有效期，不复制整套 VAD 参数表。
- profile 只提供稳定默认值，校准文件和用户显式环境变量仍可覆盖，形成
  “代码默认值 < 场景 profile < 现场校准/显式覆盖”的配置优先级。

这样既保留现场调参能力，又避免两个入口对同一个 `low_gain` 名称产生不同且无法追踪的含义。

## 12. 测试体系

关键代码：

- `tests/repository/test_repository_architecture.py`
- `tests/repository/test_repository_delivery.py`
- `tests/repository/test_repository_voice_runtime.py`
- `tests/integration/test_acceptance_cli.sh`
- `tests/integration/test_continuous_voice_control.py`
- `src/embodied_agent_core/test/`：共享领域与运行时
- `src/embodied_voice_frontend/test/`：VAD、KWS、声纹输入 Adapter
- `tests/integration/test_online_api.py`：在线 provider Adapter
- `src/embodied_offline_agent/test/`
- `src/embodied_agent_cpp/test/`
- `src/embodied_simulation/test/`
- `scripts/acceptance_test.sh`
- `scripts/showcase_release_gate.py`

设计方式：

- repository test 保证文件结构和文档入口不漂移。
- Python 单测覆盖 Agent 侧规则、会话、队列、补全。
- C++ 单测覆盖 validator、ActionScheduler、Action client contract、BT 和 pluginlib 仿真执行器。
- smoke script 覆盖 ROS 2 topic/action/launch 组合。
- `release-gate` 默认固定 5 条聚合命令，并输出 `logs/acceptance_report.json`；
  `demo-gate` 输出 `logs/demo_acceptance_report.json`，更偏现场展示证据，例如 provider preflight、
  speaker memory、连续多命令、语音导航 mock 和离线展示报告。这样演示前有一份可复查报告，
  而不是在大量 smoke 脚本中临时挑命令。
- 真实麦克风用 `continuous-live-check` 做人工辅助证据统计。

为什么这样设计：

- 机器人项目很容易“单个模块能跑，整链路断掉”。
- 分层测试能快速定位问题发生在 ASR、Agent、ActionGuard、Action、Gazebo 哪一层。
- 人工真实语音无法完全自动化，所以需要明确“人工验收标准”。

方案对比：

- 只做单测：无法证明 Gazebo 真动了。
- 只做人工演示：不可复现，回归成本高。
- 单测 + smoke + 人工验收：更适合当前工程规模。

## 13. 语音目标点导航与多目标点巡航

关键代码：

- `src/embodied_agent_core/embodied_agent_core/navigation_phrases.py`
- `src/embodied_agent_core/embodied_agent_core/command_nlu.py`
- `src/embodied_agent_core/embodied_agent_core/command_fallback.py`
- `src/embodied_agent_interfaces/msg/RobotCommand.msg`
- `src/embodied_agent_core/embodied_agent_core/ros_action_transport.py`
- `src/embodied_agent_cpp/src/action_validator.cpp`
- `src/embodied_simulation/include/embodied_simulation/nav2_places.hpp`
- `src/embodied_simulation/config/places.yaml`
- `src/embodied_simulation/rviz/voice_nav2_demo.rviz`
- `src/embodied_simulation/src/simulation_control_node.cpp`
- `src/embodied_simulation/src/nav2_robot_executor.cpp`
- `src/embodied_simulation/launch/voice_nav2_turtlebot3.launch.py`
- `scripts/continuous_nav2_voice_control.sh`
- `scripts/audit_nav2_demo_assets.py`
- `scripts/publish_nav2_initial_pose.py`
- `tests/integration/test_navigation_sequence.py`
- `tests/integration/test_continuous_navigation_queue.py`
- `tests/integration/test_nav2_bridge_sequence.py`
- `tests/integration/test_nav2_turtlebot3_voice.py`
- `tests/integration/test_continuous_nav2_voice_control_script.py`

设计方式：

- Agent 层只解析“去哪里”和“经过哪些点”，输出 `navigate_to` 或 `follow_waypoints`，不直接写坐标。
- `navigation_phrases.py` 用语义地点词表把“门口/书桌/起点”等口语映射为 `door/desk/home`。
- `RobotCommand.msg` 新增 `NAVIGATE_TO / FOLLOW_WAYPOINTS / CANCEL_NAVIGATION` 和 `target/waypoints/number_of_loops` 字段。
- `ActionGuard` 继续作为安全边界：校验地点白名单、巡航点数量、循环次数，再转换为 typed command。
- `places.yaml` 维护语义地点到地图坐标的映射。
- `tests/repository/test_repository_voice_runtime.py` 会检查 Agent 地点词表、ActionGuard 白名单和
  `places.yaml` 的 canonical place 完全一致，避免“语音能解析但 Nav2 不认地点”的漂移。
- mock/Gazebo executor 用“运动窗口”模拟导航和巡航，确保 `/cmd_vel`、Action feedback、Action result 可观测。
- `Nav2RobotExecutor` 作为 pluginlib 插件调用 Nav2 `NavigateToPose / FollowWaypoints` action；
  `nav2-bridge` 用 fake Nav2 action server 自动验证 goal 内容和底层 cancel request。
- 连续控制中，“取消导航”属于控制面指令：online/offline Agent 会清理待执行队列、唤醒
  正在等待 result 的 sequence，并立即发布 `CANCEL_NAVIGATION`，而不是排到 FIFO 尾部。
- Nav2 executor 的 goal handle 由互斥锁和 generation 保护；取消或新 goal 会递增代次，
  晚到的旧 result callback 不得清空新 goal 或覆盖新的执行状态。
- `RobotExecutor::external_action_update()` 让 Nav2 action result 反向驱动本项目
  `ExecuteRobotCommand` 的 result，避免只按本地 `duration_s` 假完成。
- `RobotExecutor::external_action_detail()` 是配套的可观测性 seam：Nav2 插件记录
  `server_unavailable`、`goal_rejected`、`aborted/canceled`、`error_code/error_msg`、
  `missed_waypoints` 等原因，`simulation_control_node` 再把这些 detail 写入
  Action feedback/result。这样现场演示失败时可以从终端直接区分“Nav2 没启动”
  和“目标执行失败”，而不是只看到笼统的 `blocked`。
- `voice_nav2_turtlebot3.launch.py` 复用官方 `nav2_bringup/tb3_simulation_launch.py`，
  再叠加本项目的 Agent、ActionGuard、typed action bridge 和 Nav2 executor。
- launch 暴露 `rviz_config_file/world/map/params_file`，默认 RViz 使用本项目的
  `voice_nav2_demo.rviz`，默认 map/world 使用项目内的 `voice_demo.yaml` 和
  `voice_demo.sdf.xacro`，方便面试时稳定展示 TF、map、scan、odom、global plan。
- `audit_nav2_demo_assets.py` 审计语义地点、Nav2 launch、RViz 配置、本地 map/world
  和验收脚本；加 `--require-local-assets` 时，可以把“项目自带固定 map/world”
  变成严格发布条件。world 文件仍通过 `model://turtlebot3_world` 复用 TurtleBot3 官方
  场景几何，这是为了降低 Gazebo/Nav2 bringup 的不确定性；项目负责维护演示入口、
  map/world 文件、目标点配置和语音到 Nav2 action 的链路。
- `nav2-turtlebot3` 重型验收会启动真实 TurtleBot3/Nav2 仿真，注入语音文本命令，
  等待目标点导航/巡航 result，并检查 `/odom` 运动证据；探针还订阅 `/map`、`/scan`，
  发布 AMCL 初始位姿并验证 `map→base_link` 定位 TF。
- 重型探针通过 `GetState` 服务等待 `bt_navigator`（巡航时还包括
  `waypoint_follower`）进入 Lifecycle ACTIVE 后才发 goal。只看到 action server 名称
  不代表它已经激活；旧做法会在启动窗口收到 `Action server is inactive` 拒绝。
- `logs/nav2_turtlebot3_voice_report.json` 保存地图元数据、激光帧数、定位 TF、Nav2
  result 数量和 odom 位移，使“真实 Nav2 跑过”成为可复查证据。
- `nav2-resilience` 在导航运行中调用 `ros_gz_sim create`，把小型静态方块动态插入
  当前全局路径前方的 inflation 区；它比较插入前后 `/plan` 到障碍中心的最小净空，
  同时要求 NavigateToPose 成功，证明不是只看到雷达或只发布了模型。
- `unreachable_zone` 是刻意放在地图外的测试目标：它通过 NLU 和 ActionGuard 白名单进入
  真正的 Nav2 planner，再要求 `aborted/error_code` 反向传播并检查 `/cmd_vel=0`。这与在
  Guard 层直接拒绝未知地点不同，前者验证的是导航失败恢复，后者验证的是输入协议安全。
- `logs/nav2_resilience_report.json` 分别记录障碍前向投影、旧/新路径净空、规划帧数、
  成功 result、不可达失败 detail 和最终速度，是动态避障/失败反馈的实际重型证据。
- `test_continuous_navigation_queue.py` 是介于普通连续队列测试和真实 Nav2 重型测试之间的
  自动回归：它验证一次唤醒后，多目标点导航和巡航命令都能进入连续队列，并按 request_id
  对应到 ROS 2 Action result；同时验证运行中语音取消会抢占，而不是排队等待。
- `continuous_nav2_voice_control.sh` 面向现场真实麦克风演示：它在 TurtleBot3/Nav2
  bringup 之上打开在线/离线 Agent 的连续语音模式，让用户一次唤醒后连续说多个目标点命令。
- `publish_nav2_initial_pose.py` 在演示启动后重复发布 AMCL `/initialpose`，降低现场
  “Nav2 已启动但机器人还没有定位”的失败概率。
- 真实 Nav2 bringup 前需要给 AMCL 发布 `/initialpose`，否则 map->odom/base_link TF
  不成立；导航 action 也要使用较长 `nav_action_timeout_s`，不能沿用普通动作 12 秒超时。

为什么这样设计：

- 先做语义地点，而不是直接语音转坐标，可以减少 ASR/LLM 的自由度，方便测试和演示。
- 新增强类型字段，而不是继续塞字符串字段，可以体现 ROS2/C++ 工程能力，也让后续 Nav2 bridge 更自然。
- 把真实 Nav2 作为 executor 插件，避免把导航细节侵入 Agent、ActionGuard 和测试。
- 用 external result seam 连接 Nav2 与本项目 Action，能体现“长动作可反馈、可取消、可等待结果”，
  而不是仅发布一个 topic 后马上认为成功。
- 失败 detail 由 executor 层产生，而不是让 Agent 猜测，是因为 planner/controller/localization
  的真实状态属于导航运行时；语音层只负责把自然语言变成目标，不能把底层故障伪装成语义错误。
- AMCL 初始位姿和长动作超时放在验收/launch 层处理，而不是塞进 Agent，保持“语音语义层”和
  “导航运行时状态层”职责分离。
- 连续麦克风 Nav2 演示脚本保留 `VOICE_CONTROL_PROFILE`、VAD endpoint、ASR commit delay、
  session timeout 和 queue size 参数，原因是导航命令更长、更容易被噪声或尾部漏识别影响；
  把这些参数显式打印出来，比“没反应时猜原因”更适合工程验收。
- Nav2 自己会发布 `/cmd_vel`，所以 `RobotExecutor::publishes_cmd_vel()` 允许 Nav2 插件禁止
  simulation node 周期性发布速度，避免两个控制器抢同一个速度话题。

方案对比：

- 直接让 LLM 输出 `{x,y,yaw}`：灵活但不稳定，且每个地图都要改 prompt；本项目更适合展示工程闭环，所以先固定语义地点。
- 只做 fake Nav2 bridge：速度快、CI 稳定，但不能证明 controller server 真的驱动机器人；因此本项目同时提供 `nav2-turtlebot3` 重型验收入口，演示前单独跑。
- 只做文本注入 Nav2 验收：自动化更稳，但不能覆盖真实麦克风的 ASR/session/queue 体验；因此新增 `continuous-nav2-offline/online` 作为人工演示入口。
- 只做字符串 topic：实现快，但难体现可取消、带反馈、可测试的 ROS 2 Action 能力。

## 14. 真实 SLAM 回访、回环约束与后端 A/B

关键代码：

- `src/embodied_slam/src/gtsam_scan_solver.cpp`
- `scripts/evaluate_slam_trajectory.py`
- `scripts/analyze_openloris_revisits.py`
- `scripts/rank_openloris_revisit_sequences.py`
- `scripts/evaluate_loop_constraints.py`
- `scripts/analyze_slam_degradation.py`
- `scripts/compare_openloris_backends.py`
- `scripts/compact_openloris_rosbag.py`
- `scripts/build_openloris_loop_sweep_configs.py`
- `scripts/compare_openloris_loop_sweep.py`
- `scripts/analyze_loop_frontend_trace.py`
- `src/embodied_slam/src/instrumented_async_slam_toolbox_node.cpp`
- `src/embodied_slam/src/loop_frontend_diagnostics.cpp`
- `src/embodied_slam/src/gtsam_graph_optimize.cpp`
- `scripts/run_gtsam_robust_kernel_ablation.py`
- `scripts/run_gtsam_loop_consistency_ablation.sh`
- `src/embodied_slam/config/openloris_loop_sweep.json`
- `src/embodied_slam/config/openloris_office1_7_annotations.json`

设计方式：先用真值定义“相隔足够久后再次进入同一位置/朝向容差”的回访采样点，
再按时间间隔聚合为事件，避免提高采样率就虚增回环机会。轨迹层检查估计轨迹是否保持回访几何；
图优化层则在 GTSAM `ScanSolver::AddConstraint` 记录前端已经接受的边，再用 Karto 原生
`end_closure.scan_index` 选出真正的闭环边。仅靠 node id 间隔会把局部图连接误判成 loop，因此
不再用于正式指标。边正确性比较测得相对位姿与真值相对位姿，回访半径只负责聚合事件；这两层
分开后才能正确计算 accepted-edge precision、false-loop rate 和事件 recall。

真值来源也分层：office 使用独立 OptiTrack；market/corridor 使用官方离线 LiDAR SLAM。后者能
提供更长的轨迹，但与在线 Hokuyo 输入并非完全独立，不能混称为 mocap 证据。下载大型 bag 前，
`rank_openloris_revisit_sequences.py` 会同时计算方向敏感回访和 360° LiDAR 位置回访；正式推荐
还要求路径 ≥100 m、时长 ≥120 s、回访两端相隔 ≥60 s。轨迹层排序最高的 `market1-3`
（约 294 s / 221.8 m / 1 个同向长回访）在完整 bag 契约中缺少 `/scan`，所以不能用于当前
2D 管线。加入传感器 profile 后，正式推荐变为 `corridor1-1`（约 272.5 s / 220.1 m / 2 个
位置长回访）。它是反向穿越同一走廊，360° LiDAR 仍有完整几何重叠，因此采用 180° 位置口径；
报告必须同时展示 yaw 范围，不能把它描述为同向视觉回环。

为什么分两层：机器人依靠较好的里程计和局部 scan matching，也可能在短路径上回到原处；最终
ATE 较低或回访恢复率较高，并不证明前端真正检测并接受了回环。`office1-7` 就给出了反例：
在旧的 10 秒宽松定义下最终轨迹恢复 2/2 次事件，但 46 条 accepted graph edge 全是相邻边，
非局部回环为 0；按正式 60 秒长回环门槛，它没有真值事件。

退化区间也不从速度阈值猜语义。工具按时间抽取 RGB 联络表，人工复核后才标注玻璃隔断和动态人员
遮挡；没有长走廊证据就记录 negative evidence。Ceres/GTSAM A/B 固定 bag、前端参数、时间窗和
评估器，只替换 `ScanSolver`，并检查匹配数、时间覆盖、运动类别及人工区间样本完全一致。

方案对比：只报告 ATE/RPE 能评价最终轨迹，但解释不了回环前端行为；解析 INFO 日志容易受版本和
文案影响；结构化 JSONL 图边可与真值重放关联。现在的 InstrumentedAsynchronousSlamToolbox 在
`addScan()` 这个同步 Karto 窄接口外包一层：先让上游完成原始处理，再通过公开 graph/mapper API
复算 `FindPossibleLoopClosure` 的候选 chain 拓扑，并用 `MapperLoopClosureListener` 收集原生
coarse/fine response、variance、reject 和 closure callback。它不修改匹配结果，也不伪造约束。
Karto 的候选函数本身是 private，因此报告明确标为 `replicated_karto_rule`；这比复制或修改
第三方源码更易升级，但仍不能当作候选函数内部逐行执行轨迹。

为了让真实阈值消融可重复，项目没有复制 1.43 GB 原包做六份实验，而是流式生成只含三个 SLAM
契约 topic 的约 5 MB ROS 1 bag。派生 `source.json` 同时保存原包/派生包 SHA256、消息数和时间窗，
实验 manifest 再绑定参数文件哈希。runner 默认只复用“bag 哈希和配置哈希都一致”的已完成结果，
避免断点续跑时混入旧参数。

实测 baseline、短 chain、低 response、宽 search、组合放宽和 extreme 诊断配置都得到 449 个
匹配位姿，ATE 约 9.96～10.02 cm；每组 46 条 accepted edge 全是相邻边。即使 extreme 把 chain
降到 1、响应阈值降到 0.05，也没有非局部边。这说明继续优化 GTSAM 的鲁棒核不会提升回环召回，
因为非局部约束尚未进入后端。新增诊断在 baseline 观察到 47 个实际图节点：前 8 个为
`insufficient_history`，其余 39 个为 `all_geometric_neighbors_near_linked`，coarse/fine check 均为
0。失败点因此进一步收窄到“候选 chain 生成前的 near-linked 排除”，不是 response、variance 或
GTSAM 鲁棒核。下一步应选择时间跨度和空间回访更大的序列，或在不破坏上游语义的前提下研究
near-linked 图遍历半径，而不是继续盲目降低 scan matcher 阈值。

`corridor1-1` 随后证明前端确实会接受真假混合的 closure，因此后端鲁棒性才成为有意义的问题。
为排除异步回放每轮 closure 数不同的干扰，`GtsamScanSolver` 将分批工作集之外再维护一份按
node/edge 去重的证据图；四种配置只读取同一个 SHA256 快照。1834 节点/2751 约束实测中，
Gaussian、Huber-all、Huber-non-local、Cauchy-non-local 的 ATE 分别为
1.8235/1.4081/1.4626/1.2236 m。Cauchy 重尾损失对大残差降权更强，但 node ID 间隔只是
non-local 启发式，不是 Karto closure 真值；这项优化降低错误边破坏程度，却不会找回漏检回环。

进一步的 consistency gate 位于鲁棒核之前：`violates_consistency_gate()` 用优化前两节点计算预测
相对位姿，并与候选边测量比较 SE(2) 平移/偏航创新量。它和“用真值筛边”有本质区别——运行时
只读当前图，因此能部署；真值只在离线实验结束后评价 ATE/RPE。固定图上 2 m / π/4 阈值拒绝
23 条边，Cauchy ATE 从 1.2236 m 降到 1.1713 m。不过，强回环本来就是为了纠正累计漂移：若当前
图错得超过阈值，硬门控会把最有价值的真回环拒掉。因此工程上采用“明显异常才硬拒绝、中等异常交给
鲁棒核、默认关闭等待多序列验证”的分层策略，而不是把一次消融最优参数直接写成生产默认值。

## 15. 动态障碍运动模型与同场景消融

关键代码：

- `src/embodied_navigation/include/embodied_navigation/dynamic_obstacle_tracker.hpp`
- `src/embodied_navigation/src/dynamic_obstacle_tracker.cpp`
- `src/embodied_navigation/src/dynamic_obstacle_model_benchmark.cpp`
- `src/embodied_navigation/src/predicted_obstacle_layer.cpp`
- `src/embodied_navigation/config/dynamic_obstacle_crossing_scenario.json`
- `scripts/compare_dynamic_navigation_models.py`
- `scripts/verify_dynamic_obstacle_ablation.py`
- `tests/integration/test_predicted_dynamic_obstacle_navigation.py`

设计方式：ROS 节点只把 `PoseArray` 转成观测并调用 `DynamicObstacleTracker::update()`，运动模型、
协方差和 IMM 模型概率全部藏在 PImpl 中。关联时使用模型预测位置而不是最后一次原始观测，短时
漏检继续发布预测，但 `last_seen` 不前移，TTL 到期仍会删除轨迹。输出继续使用原有 typed
`DynamicObstacleArray`，所以 costmap plugin 和 Nav2 不需要知道选择了哪种滤波器。

为什么这样设计：消融实验必须只替换一个变量。如果为每种算法复制 ROS 节点、topic 或 launch，
差异会混入 QoS、时间戳和调度噪声；统一 seam 让四种 Adapter 接收相同观测，并让同一个
`PredictedObstacleLayer` 消费结果。tracker-level 报告负责比较 RMSE，Gazebo/Nav2 报告负责证明
lethal cost、重规划、到达和最终零速，两层证据互不替代。

重型 A/B 的“同场景”不能只靠 for 循环保证。场景 JSON 固定目标点、观测位置、时间间隔、预测
时域和阈值；每份报告绑定场景、地图 YAML+PGM 和 Nav2 参数哈希，比较器先验证 provenance 再
汇总指标。这种设计比在测试函数里硬编码四组坐标更容易审阅，也能阻止断点重跑时混入旧地图。
局限同样写进报告：目前只把确定性 `PoseArray` 注入跟踪器，尚未模拟物理行人的碰撞体、传感器
遮挡和检测器误差，所以它证明的是预测层到规划控制的闭环，而不是感知算法的真实准确率。

方案区别：current-only 没有运动先验；平滑 CV 低成本但无法表达模式切换；单一 Kalman 假设固定
过程模型；IMM 通过 Markov 转移概率、状态/协方差交互和观测似然在低运动与机动模型之间切换。
当前 IMM 在长序列综合误差最低，但短序列启动偏保守，说明选择模型还要考虑观测窗口和业务风险，
不能只看一个总 RMSE。

## 16. 面试讲法建议

可以用这条主线介绍项目：

> 我做的是一个 ROS 2 机器人智能语音控制系统。前端用 ASR 把语音转文本，Agent 负责唤醒、连续会话、命令归一化、LLM 动作解析和 TTS。动作不会直接控制机器人，而是先进入 C++ ActionGuard 做校验和限幅，再转换成自定义 RobotCommand 和 ROS 2 Action。仿真侧用 BehaviorTree.CPP 编排安全检查、执行和结果确认，用 pluginlib 切换 mock/Gazebo executor。最终在 Gazebo/TurtleBot3 里验证 `/cmd_vel` 和 odom 变化。

强调点：

- 不是只调 API，而是打通了 ROS 2 端到端控制链路。
- 不是 LLM 直接发速度，而是有 ActionGuard 和强类型 Action。
- 不是只写 demo，而是有连续语音、队列、急停、验收脚本和测试体系。
- 不是复杂导航项目，当前重点是语音到动作到仿真控制的闭环。
