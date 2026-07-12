# 版本记录与路线图

本文档记录项目阶段性演进、当前完成度和后续路线。详细架构见 [ARCHITECTURE_AND_KNOWLEDGE.md](ARCHITECTURE_AND_KNOWLEDGE.md)，关键技术学习笔记见 [LEARNING_NOTES.md](LEARNING_NOTES.md)。

## 1. 阶段性版本记录

| 阶段 | 主要目标 | 结果 |
| --- | --- | --- |
| 初始骨架 | 创建 ROS 2 workspace，搭建在线 Agent、动作 topic、stub 节点 | 完成无密钥 mock 链路 |
| 在线 Agent | 接入在线 ASR/LLM/TTS，设计 prompt、记忆、动作格式 | 完成在线接口 smoke 与动作解析 |
| C++ 化与安全网关 | 将适合 C++ 的 ROS 节点迁移/新增为 C++，加入 ActionGuard | 完成动作校验、限幅、强类型转换 |
| 离线 Agent | 接入 Sherpa/llama.cpp/Sherpa-TTS 适配与 mock 链路 | 完成离线结构、双缓冲、延迟统计、smoke 入口 |
| 仿真控制 | 接入 Gazebo/TurtleBot3，打通 `/cmd_vel` 与 odom 验收 | 完成语音到 Gazebo 运动闭环 |
| Nav2 风格规范化 | 引入自定义 msg/action、Lifecycle、BehaviorTree、pluginlib | 完成 typed Action、BT 编排、mock/Gazebo executor |
| 丰富演示动作 | 增加前进、后退、转向、绕圈、正方形、演示序列 | 完成动作秀和安全演示 |
| 连续语音控制 | 一次唤醒后连续说多条命令，支持队列和急停 | 完成 continuous offline/online 验收入口 |
| 真实语音稳定性 | 修复尾部漏识别、重复识别、filler、queue_full 可观测性 | 完成 VAD profile、commit delay、短命令补全、monitor |
| 阶段性文档收尾 | 整理 README、验收文档、学习笔记、关键中文注释 | 当前阶段 |
| 轻量 NLU 多命令 | 识别一句 ASR final 内的多个动作，并保证队列顺序 | 新增 CommandNLU、batch 可观测性、request_id/result 关联 |
| 语音导航与巡航 | 支持语音目标点导航、多目标点巡航，并接入 typed Action 与 Nav2 bridge | 新增 navigate_to/follow_waypoints/cancel_navigation 协议、NLU、ActionGuard 校验、navigation-demo 和 nav2-bridge |
| Nav2 bringup 入口 | 复用官方 Nav2 TurtleBot3 仿真 launch，接入本项目语音控制链路 | 新增 voice_nav2_turtlebot3.launch.py、nav2-preflight 和 nav2-turtlebot3 重型验收 |
| Nav2 result 闭环 | 用 Nav2 action result 驱动本项目 ExecuteRobotCommand result | 新增 RobotExecutor external_action_update seam，避免导航 goal 按本地 duration 假完成 |
| 真实 Nav2 验收修复 | 跑通 TurtleBot3/Nav2 目标点导航与多目标点巡航 | 修复官方 launch 布尔参数、AMCL initialpose 和导航长动作超时；`nav2-turtlebot3` PASS |
| 真实麦克风 Nav2 连续导航 | 支持一次唤醒后连续说多个目标点/巡航命令并进入 Nav2 队列执行 | 新增 `continuous-nav2-offline/online`、AMCL initialpose 辅助脚本和 live-check 入口 |
| 连续导航队列回归 | 自动验证连续会话中目标点导航与多目标点巡航不会丢队列 | 新增 `continuous-navigation`，覆盖多目标点 NLU、队列元数据和 request_id/result 关联 |
| Nav2 韧性重型验收 | 用真实 Gazebo/雷达/costmap 证明动态重规划和不可达失败反馈 | 新增 `nav2-resilience`，动态插入前方障碍、比较全局路径净空，并验证地图外目标 aborted 与停车 |
| Nav2 现场验收增强 | 让真实麦克风辅助计分更贴近导航目标 | `continuous-nav2-live-check` 额外要求出现 `navigate_to` 与 `follow_waypoints` |
| 真实麦克风验收留证 | 让现场验收结果可保存、可复查 | `CONTINUOUS_LIVE_CHECK_REPORT=...` 可导出 live-check 证据报告 |
| 真实麦克风报告复核 | 让现场报告可以脱离仿真环境二次判定 | 新增 `continuous-live-report` / `continuous-nav2-live-report` |
| 一键式 Nav2 现场留证 | 降低真实麦克风 Nav2 验收操作复杂度 | 新增 `continuous-nav2-evidence`，单终端启动控制、计分、保存报告并清理进程 |
| 一键留证 dry-run | 让现场验收脚本可自动测试、可提前检查参数 | `CONTINUOUS_NAV2_EVIDENCE_DRY_RUN=true` 打印控制/计分命令但不启动仿真 |
| 成熟 VAD 自动选择 | 连续语音默认优先使用可用的 Silero VAD，缺依赖时降级 energy | `VAD_PROVIDER=auto`、`voice_provider_preflight.py` 自动解析、普通/Nav2 连续脚本统一 |
| 自然多目标导航话术 | 提升真实语音目标点/巡航表达容错 | 支持“先去门口再去书桌最后回起点”“巡逻门口书桌起点”，同时保留两目标语句拆成多个 `navigate_to` 入队 |
| 自然导航验收入口 | 将自然多目标话术纳入 ROS pipeline 回归 | 新增 `continuous-navigation-natural`，覆盖自然话术到 `follow_waypoints` 队列执行 |
| Sherpa-ONNX ASR-only 部署 | 开始真实部署离线 ASR 推理框架，先隔离验证 ASR 层 | 新增 `setup_sherpa_asr_runtime.sh`、`sherpa_asr_smoke.py`、`sherpa-asr-preflight/smoke` 验收入口 |
| Sherpa-ONNX 离线完整链路验证 | 验证真实 Sherpa 语音模型进入 ROS2 typed Action 控制闭环 | 新增 `offline-sherpa-typed`，覆盖 Sherpa-TTS 音频、ZipFormer ASR、Offline Agent、ActionGuard、ExecuteRobotCommand、`/cmd_vel` |
| 离线 TTS 版本收口 | 固定 llama.cpp / SummerTTS / sherpa-onnx 版本并补充低延迟 gate | 新增 `OFFLINE_RUNTIME_VERSIONS.md`、`offline-runtime-versions`、`offline-latency` |
| SummerTTS 服务化 | 将 SummerTTS 从命令行 provider 升级为常驻 C++ ROS service | 新增 `SynthesizeSpeech.srv`、`summer_tts_service`、`tts_provider:=summer_ros`、`summer-tts-service` |
| SummerTTS 短文本缓存 | 优化“收到/好的/正在执行”等重复反馈的服务延迟 | `SynthesizeSpeech.srv` 增加 `cache_hit`，`summer_tts_service_probe.py` 输出首轮/缓存命中耗时 |
| 指令解析评测增强 | 把 deterministic parser 证据从 seed 样例扩展为代表集 | `robot_instruction_eval.jsonl` 扩展到 43 条，覆盖速度/距离/角度/时长/地点槽位及长动作分段，`instruction-parser-eval` 输出分 tag 指标与失败用例 |
| 求职展示版收口 | 固定演示路径、汇报稿、代码走读地图和发布门禁 | 新增 `PROJECT_PRESENTATION_15MIN.md`，README 指向阶段发布 gate |
| Nav2 演示资产本地化 | 减少对官方 `tb3_sandbox` map/world 入口的展示依赖 | 新增 `voice_demo.yaml`、`voice_demo.sdf.xacro`，`nav2-assets` 审计本地 map/world/RViz |
| 成熟 VAD 预检闭环 | 降低真实麦克风现场排障成本 | `provider-preflight` 输出 `recommendations`，连续语音启动时提示 WebRTC/Silero setup 命令 |
| WebRTC VAD 运行时验收 | 让成熟 VAD 不只停留在 preflight | 新增 `webrtc-vad-sidecar`，验证 WebRTC VAD sidecar 可启动并接管 endpoint |
| Sherpa KWS 部署闭环 | 让声学唤醒路径可复制验收 | `setup_voice_kws_runtime.sh sherpa` 生成 `logs/sherpa_kws.env`，`sherpa-kws-sidecar` 验证真实 KeywordSpotter 启动 |
| KWS 阈值校准闭环 | 让现场 KWS 分数能直接变成下一轮参数 | `voice-calibration-report` 写出 `OPENWAKEWORD_THRESHOLD` / `LIVEKIT_WAKEWORD_THRESHOLD` 推荐值 |
| Silero ONNX 轻量运行时 | 让成熟 VAD 不依赖 PyTorch 并具备真实推理证据 | 固定 v6.2.1 模型/哈希，纯 ONNX state/context 推理，ROS endpoint 与延迟报告通过 |
| 离线性能证据收口 | 统一 ASR、LLM、TTS、tokens/s 和真实 E2E 指标 | 增加运行时预热、单槽 prompt cache、`offline-voice-e2e-report` 和严格证据审计；本轮端到首 PCM 多次运行约 1.32–2.11s |
| 动作控制面全强类型化 | 删除 Agent→ActionGuard 的 JSON 适配层，让候选、受信命令、反馈和结果都使用自定义 ROS 2 接口 | 新增 `RobotCommandFeedback` / `RobotCommandResult`，Agent 直接发布 `RobotCommand`，C++ ActionGuard 直接校验字段；JSON 只保留在日志、指标和硬件协议边界 |
| C++ Action 调度收敛 | 将受信动作的执行顺序、Action Client、优先取消和状态观测从 Python 收敛到 C++ | 新增可独立测试的 `ActionScheduler`，组合动作批量进入 C++ FIFO；显式 `priority` 区分急停与计划 STOP，并增加取消 watchdog、稳定错误码、`/diagnostics` 和 `cpp-action-scheduler` 验收 |
| 命令生命周期中间件强类型化 | 删除队列/执行事件的 `String + JSON` ROS 契约并统一 QoS | 新增 `CommandContext`、`CommandQueueEvent`、`CommandExecutionEvent`；online/offline、monitor、live-check 和集成探针统一使用 typed msg；命令事件 reliable，当前状态 transient-local |
| 语音控制面事件强类型化 | 将唤醒、识别反馈和 NLU 解析从通用字符串中拆出 | 新增 `WakeEvent`、`RecognitionFeedback`、`NluParseEvent` 及槽位/改写子消息；识别状态与 NLU 动作序列分 topic，online/offline 和验收探针使用同一转换边界 |
| Agent 公共控制面收敛 | 删除 online/offline 中重复的会话入口、队列组件初始化和 typed publisher 实现 | 新增无 ROS 依赖的 `AgentControlPlane` 与独立 `RosAgentEventPublisher` Adapter；统一参数映射、归一化、补全、重试、急停/导航取消决策和 batch id |
| 运行状态中间件强类型化 | 清理音频、VAD/KWS、仿真状态、ACK 和 BT 状态的 `String + JSON` 契约 | 新增 7 个运行状态 msg 与统一转换 Adapter；VAD/KWS provider、C++ 仿真/硬件节点、monitor 和验收探针共享同一 schema，JSON 仅保留为报告文件格式 |
| 声纹记忆模块收敛 | 删除 online/offline 重复的记忆命令状态分支和声纹 JSON topic | 新增 `MemoryCommandService` 深模块及 3 个声纹 typed msg；身份门槛、偏好生命周期、录入请求和 interaction 记录使用同一实现 |
| 仿真动作运行时收敛 | 缩小 Lifecycle 节点职责，消除定时动作、Nav2 result 与 BT 的平行状态机 | 新增 `ActiveActionRuntime`，统一进度、取消、超时、外部 result 和 BT 终态映射，并增加纯 C++ 单测 |
| 控制命令启动可靠性 | 修复 DDS discovery 完成前 Guard 发布的 volatile 动作静默丢失 | 新增有界 TTL `GuardedCommandOutbox`；scheduler 匹配后 FIFO 转发，超时/满载明确拒绝，不回放陈旧动作 |
| C++ 中间件契约收敛 | 清理跨节点散落的 QoS depth 与不一致策略 | 新增独立 `embodied_agent_middleware` 包，统一 command/event/state/sensor/audio/diagnostics QoS，并迁移控制主链路 |
| 系统就绪状态收敛 | 替代 launch/test 中分散的固定 sleep、topic graph 猜测和日志字符串判断 | 新增 `ComponentHealth`、`SystemReadiness`、心跳超时聚合器和 profile 化启动门禁；保留音频/仿真数据质量探针 |
| Agent 参数与 launch 契约收敛 | 消除 online/offline 节点、YAML、Gazebo/Nav2 launch 中重复默认值和转发映射 | 新增共享参数 schema、ROS range/enum 描述、启动前校验、只读快照与组合 launch 转发契约；provider YAML 仅保留模型配置 |
| Agent 并发运行时收敛 | 消除端点 timer、busy/worker 和多命令 NLU 入队的双份状态机 | 新增 `AsrEndpointRuntime`、`AgentExecutionRuntime` 和 `CommandEnqueueDecision`；统一异常隔离、busy 复位、batch metadata 与关闭时 timer/cancel 语义 |
| LLM 流式协议收敛 | 消除 online/offline 的 token parser、TTS 分句与动作选择双份实现 | 新增 `StreamingTurnRuntime/Result`；统一格式错误回退、确定性动作优先级、语义安全阻断和记忆动作口径，provider 仅保留 TTS/latency adapter |
| 用户上下文一致性收敛 | 消除身份、画像 prompt、偏好和低置信度写保护的双份节点逻辑，并修复异步声纹切换竞态 | 新增 `UserContextRuntime/Snapshot`；命令入队时冻结身份/偏好，prompt、动作与 interaction 共用同一快照；预解析动作恢复归入 `AgentControlPlane` |
| Agent Lifecycle 资源治理 | 让 online/offline 的生命周期状态对应真实 provider、线程和 publisher，而非只保留进程级启停 | 两个 Agent 升级为 `LifecycleNode`；configure/activate/deactivate/cleanup/on_error 统一资源边界，managed publisher、协作取消、安全 STOP、STOPPED health、manager 依赖顺序和 cleanup 后重建均有自动验收 |
| Agent ROS I/O 契约收敛 | 消除 online/offline 节点重复接线、topic 字符串和 QoS 漂移 | 新增 `AgentRosIo`、不可变 `AgentTopicContract` 与 `audio_qos`；节点只注入 callback，PCM best-effort、控制 reliable、状态 latched，并在 inactive 关闭健康心跳 |
| Agent turn 指标强类型化 | 删除在线/离线双 topic 与 `String + JSON` 指标协议 | 新增 `AgentTurnMetrics` 和唯一 `metrics_transport.py`；统一 `/agent/metrics`，source 区分模式，NaN/三态 target 表达缺失值，监控与验收共享转换 Adapter |
| Agent Lifecycle 编排收敛 | 消除 online/offline 对 active/stopping、endpoint、execution 和安全停机顺序的双重所有权 | 新增组合式 `AgentLifecycleRuntime`；统一 bind/activate/deactivate/release/shutdown、priority STOP 与 quiescence 报告，provider 仅注入输入启停 hook |
| Agent 包依赖收敛 | 消除 offline 复用 online 内部业务模块和语音 sidecar 形成的反向依赖 | 新增 `embodied_agent_core` 与 `embodied_voice_frontend`；公共领域/编排/记忆/transport 和 VAD/KWS/声纹 Adapter 分别归位，online/offline 只保留各自 provider |
| Python/C++ QoS 语义对齐 | 删除 VAD/KWS/声纹节点手写 QoS 和跨语言命名漂移 | Python/C++ 统一 command/event/state/sensor/audio/diagnostics 六类 profile；KWS score 与 PCM 使用 best-effort，身份/健康使用 transient-local，控制和事件保持 reliable + volatile |

## 2. 当前完成度结论

当前项目已经达到“语音输入 → 大模型/规则动作解析 → ROS 2 安全校验 → Gazebo 仿真控制”的主链路目标。

已具备的展示点：

- ROS 2 C++ 节点：音频前端、ActionGuard、typed action bridge、仿真执行层；动作控制面不再依赖 JSON 字符串解析。
- Python Agent：在线/离线 provider、连续语音会话、命令队列、LLM/TTS 编排。
- 工程化接口：自定义 msg/action、Lifecycle、BehaviorTree.CPP、pluginlib。
- 演示能力：真实麦克风连续语音、多动作序列、急停抢占、Gazebo 运动验证。
- 多命令能力：一条 ASR final 可被轻量 NLU 解析为多个队列项，并按 ROS 2 Action result 顺序执行。
- 导航演示能力：支持“去门口”“前往书桌”“依次去门口、书桌、起点”等语音目标点/多点巡航命令，并通过 typed Action 驱动仿真 executor、Nav2 action bridge 或完整 TurtleBot3/Nav2 bringup。
- 测试体系：单元测试、集成 smoke、Gazebo 验收、真实麦克风辅助统计。
- 汇报材料：已补充 15 分钟项目汇报与代码走读稿，便于按链路讲解关键文件和技术取舍。

需要谨慎表述的边界：

- 当前硬件控制是预留/mock，不是实体机器人完整验收。
- 当前已提供完整 TurtleBot3/Nav2 重型验收入口，但地图构建、复杂目标点规划和更复杂场景仍是后续增强。
- 离线 LoRA 训练、量化指标可以作为规划和接口说明，不应夸大为已复现完整训练结果。
- openWakeWord、LiveKit WakeWord 仍是可选 seam/preflight；Silero VAD 已有轻量 ONNX 真实运行时，
  但模型仍保持可选下载，CI 不强制携带大模型资产。

## 3. 当前最有价值的验收证据

基础自动验收：

```bash
bash scripts/acceptance_test.sh mock
```

连续语音自动验收：

```bash
bash scripts/acceptance_test.sh continuous-endpoint
bash scripts/acceptance_test.sh continuous-mock
bash scripts/acceptance_test.sh continuous-multi-command
bash scripts/acceptance_test.sh continuous-queue-full
bash scripts/acceptance_test.sh voice-readiness
```

语音导航/巡航验收：

```bash
bash scripts/acceptance_test.sh nav2-stage
bash scripts/acceptance_test.sh navigation-demo
bash scripts/acceptance_test.sh continuous-navigation
bash scripts/acceptance_test.sh nav2-bridge
bash scripts/acceptance_test.sh nav2-preflight
bash scripts/acceptance_test.sh nav2-turtlebot3
```

Sherpa-ONNX ASR-only 验收：

```bash
bash scripts/acceptance_test.sh sherpa-asr-preflight
bash scripts/acceptance_test.sh sherpa-asr-smoke
bash scripts/acceptance_test.sh offline-sherpa-typed
```

真实麦克风 Nav2 连续导航验收：

```bash
bash scripts/acceptance_test.sh continuous-nav2-offline
CONTINUOUS_LIVE_CHECK_DURATION=240 bash scripts/acceptance_test.sh continuous-nav2-live-check offline
```

Gazebo 验收：

```bash
bash scripts/acceptance_test.sh gazebo
bash scripts/acceptance_test.sh gazebo-voice
bash scripts/acceptance_test.sh gazebo-voice-online
```

真实麦克风验收：

```bash
bash scripts/acceptance_test.sh continuous-offline
bash scripts/acceptance_test.sh continuous-live-check offline
```

## 4. 近期路线图

### P0：保持演示稳定

- 优先保证 `continuous-offline` 在 3～5 分钟内稳定连续控制。
- 新增固定 10 命令真实麦克风 benchmark，量化识别率、动作成功率、误触发率与延迟 P95；
  自动测试不再冒充真人长时间证据。
- 新增 `continuous-voice-evidence` 单终端入口，自动启停控制链路并显示倒计时；benchmark
  即使未达门槛也保证生成现场与汇总两份报告，避免 `set -e` 提前中断留证。
- 优先保证 `continuous-nav2-offline` 能支撑 3～5 分钟真实麦克风目标点导航/巡航演示。
- 继续完善 monitor 输出，让失败原因能直接定位到 ASR、session、queue、Action、Gazebo。
- 为常见麦克风和噪声环境补充 profile 建议。
- 阶段发布前固定运行 README 中的 release gate，并把真实麦克风/Gazebo/Nav2 结果留成可复查证据。

### P1：增强 ROS 2/C++ 求职展示价值

- 继续提高 C++ 节点比例：安全校验、仿真执行、协议层、诊断层优先 C++。
- 补充更多 C++ 单元测试和 launch test。
- 将关键设计整理成可讲的架构图和面试问答。

### P2：补齐端侧部署故事

- 固化离线模型下载、量化、启动 llama.cpp server 的流程。
- 增加离线 benchmark 报告模板。
- 继续评估 SummerTTS 量化、缓存或更快声码器；当前 `summer_ros` 证明服务化封装，不作为低延迟默认路径。

### P3：增强真实 Nav2 导航栈

- 在已有 `voice_nav2_turtlebot3.launch.py` 基础上增加保存地图、AMCL/SLAM、目标点巡航场景资产。
- 增加更稳定的真实 Nav2 odom/goal-result 统计报告，区分 planner/controller/behavior tree 失败原因。
- 明确 LoRA 训练数据集格式和复现实验入口。

### P3：可选增强

- 接入真实声学 KWS，例如 sherpa-onnx keyword spotting 或 openWakeWord。
- 接入 WebRTC AEC/NS，改善扬声器回声环境。
- 引入更接近 Nav2 的行为树 XML 和复杂安全策略。
- 接真实硬件底盘或串口设备，完成 UART/SPI 实体验收。

## 5. 不建议近期优先做的事情

- 过早引入完整导航栈、地图和路径规划：会稀释当前“语音到动作控制闭环”的主线。
- 大规模重写仓库结构：当前更需要稳定验收和文档清晰。
- 依赖复杂声学模型作为默认链路：会提高部署门槛，影响演示可复现性。

## 6. 项目介绍版本

具身智能机器人智能语音交互与仿真控制系统

项目描述：设计并实现机器人智能语音交互系统的在线与离线 Agent 链路，打通从语音输入、ASR 识别、大模型/规则动作解析、动作安全校验到 ROS 2/Gazebo 仿真控制的端到端流程。项目引入自定义 ROS 2 msg/action、C++ ActionGuard、BehaviorTree.CPP 和 pluginlib 执行器，支持真实麦克风连续语音控制、多命令队列、急停抢占和仿真运动验收。

主要工作：

- 在线流式 Agent：接入在线 ASR/LLM/TTS provider，设计系统 prompt、记忆管理、动作输出格式、fallback parser 与动作回调，实现 ASR final 到动作候选和 TTS 反馈的流式交互。
- 离线 Agent：预留 Sherpa-onnx ZipFormer ASR、llama.cpp、Sherpa-TTS 的端侧部署结构，设计双缓冲和延迟统计，支持 mock 与真实模型 smoke 验收。
- 连续语音控制：实现 wake/session gate、重复 ASR final 过滤、filler 过滤、命令队列、TTL、急停抢占、短命令补全和现场 monitor，提高真实麦克风长时间控制稳定性。
- ROS 2/C++ 控制链路：基于 C++ 编写 ActionGuard、typed action bridge、仿真执行节点，将 LLM 动作候选转换为强类型 RobotCommand 和 ROS 2 Action，并在 Gazebo/TurtleBot3 中验证 `/cmd_vel` 和 odom 变化。
- 工程化与验收：引入 BehaviorTree.CPP 编排动作校验、安全检查、执行和结果确认，用 pluginlib 支持 mock/Gazebo executor，配套单元测试、集成 smoke、Gazebo 验收和真实麦克风人工验收脚本。
