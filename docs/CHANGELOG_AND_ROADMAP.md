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
| 阶段性文档收尾 | 整理 README、验收文档、学习笔记、关键中文注释 | 已完成主入口与分层文档收敛 |
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
| 仿真 ROS I/O 收敛 | 避免控制节点同时维护业务状态、publisher 生命周期和 DDS 细节 | 新增 `SimulationRosIo`，统一 7 个 managed publisher、命名 QoS、ACK/BT 映射与去重，节点缩减到 800 行以内 |
| executor 后端隔离 | 避免简单 Gazebo/Mock 后端和 Nav2 Action client、线程、地图加载耦合在同一编译单元 | 按 Gazebo、Mock、Nav2 拆为三个 pluginlib 实现文件，保持稳定插件名称和公共 `RobotExecutor` 契约 |
| 真实语音 profile 收敛 | 消除普通控制与 Nav2 入口各自维护 normal/quiet/low_gain/noisy_room 参数表造成的漂移 | 新增 `voice_control_profile.sh` 作为唯一解析器，场景只覆盖会话基线，显式环境变量仍拥有最高优先级 |
| voice frontend launch 收敛 | 消除 online/offline 对音频、VAD、KWS、声纹参数和节点的整段复制 | 新增 `voice_frontend_launch_contract.py`，统一 31 个参数和 5 个节点；在线 launch 缩至约 100 行、离线约 156 行 |
| Agent 安全部署拓扑收敛 | 避免 ActionGuard/Lifecycle manager/硬件 Adapter 顺序与参数在 online/offline 漂移 | 新增 `agent_deployment_launch_contract.py`；统一正序激活、逆序停机及 UART/SPI 类型，在线 launch 进一步缩至约 57 行、离线约 123 行 |
| 仓库契约测试分区 | 避免结构、部署、语音和仿真守卫继续堆积在 1700 行单文件 | 按 architecture/delivery/voice_runtime 拆为三组，共享只读路径工具；54 项契约保持通过并增加文件规模守卫 |
| 可度量 SLAM 闭环 | 不把启动现成建图包当作完成，量化漂移与回环修正 | 新增固定 seed 漂移注入、闭环路线、ATE/闭环误差/地图覆盖报告和 Ceres/GTSAM 同前端 A/B |
| GTSAM 后端插件 | 自己实现可替换的位姿图后端并接入真实建图流程 | 新增纯 Pose2 optimizer、协方差正定防护及 `karto::ScanSolver` pluginlib Adapter |
| 地图复用定位导航 | 证明建图产物能在新进程中用于任务执行 | 保存 5 cm 地图，完成 AMCL `map->odom`、Nav2 plan、NavigateToPose 和零速收尾 |
| 预测动态避障 | 从“检测当前障碍”升级为“估计速度并占用未来轨迹” | 新增 typed track、常速度预测深模块、Nav2 costmap layer；实测路径净空由约 0.011 m 提升到约 0.976 m |
| bringup 包分层 | 修正共享 launch contract 放在领域 core 中造成的部署依赖反向污染 | 新增 `embodied_agent_bringup`，依赖方向统一为 bringup → core/voice/C++；core 移除 launch/launch_ros 依赖 |
| 控制命令启动可靠性 | 修复 DDS discovery 完成前 Guard 发布的 volatile 动作静默丢失 | 新增有界 TTL `GuardedCommandOutbox`；scheduler 匹配后 FIFO 转发，超时/满载明确拒绝，不回放陈旧动作 |
| C++ 中间件契约收敛 | 清理跨节点散落的 QoS depth 与不一致策略 | 新增独立 `embodied_agent_middleware` 包，统一 command/event/state/sensor/audio/diagnostics QoS，并迁移控制主链路 |
| 公开数据 SLAM 评估层 | 把仿真闭环指标升级为可复用的真实轨迹评价工具 | 新增 ROS 1/2 bag Adapter、OpenLORIS 真值校验、固定尺度 SE(2) 对齐、ATE/RPE/回访/退化段报告和无下载 CI 门禁 |
| OpenLORIS 双后端回放 | 让公开 bag 直接驱动项目 SLAM，而不只导出 odom | 新增单调时钟、隔离 TF、重复帧过滤、map-frame recorder、Ceres/GTSAM 真实 A/B 入口和小 bag 双后端门禁 |
| OpenLORIS 实验可复现性 | 让大型公开数据和精度数字具备来源链 | 新增断点续传/哈希/安全解包、运动退化分段、人工标注边界和 commit/config/artifact manifest |
| OpenLORIS 真实回访证据 | 区分“轨迹回到附近”与“前端实际接受回环” | 选择 `office1-7`，按 tar 成员 range 下载并双哈希；新增真值事件聚合、GTSAM accepted-edge 日志和 false-loop/event-recall 报告 |
| Karto 回环前端可观测性 | 把“无 accepted loop”定位到候选、粗匹配、细匹配或约束插入阶段 | 新增 C++ 生命周期诊断节点、候选链规则复算、原生 matcher callback、JSONL 汇总和 manifest 绑定；office1-7 当前定位为 near-linked 排除 |
| OpenLORIS 长环路序列筛选 | 避免只看真值排名或先下载十几 GB bag 才发现传感器不兼容 | 真值包支持断点续传/缓存；批量比较 22 条轨迹；`market1-3` 因完整 bag 缺少 `/scan` 被拒绝，传感器 profile 正式选择约 272.5 s / 220.1 m、含 2 次位置长回访的 `corridor1-1`；range 与 bag 固定 commit/大小/SHA256 |
| GTSAM 鲁棒核固定图消融 | 排除异步回放前端差异，量化错误非局部边对后端的影响 | 导出 1834 节点/2751 约束去重图；同 SHA256 比较 none/Huber/Cauchy；Cauchy non-local ATE 1.2236 m，较 Gaussian 下降 32.90%，同时保留“不改善前端 precision”的边界 |
| 非局部边几何一致性门控 | 在鲁棒核前拒绝与当前图预测明显冲突的候选边，并保持运行时不依赖真值 | 固定图上拒绝 23 条约束；Cauchy + gate ATE 1.1713 m，较 Gaussian 下降 35.77%；因累计漂移可能误拒真回环，默认关闭 |
| LaserScan 双证据约束复核 | 用传感器几何补充位姿创新，避免只凭当前图硬门控 | 原始 `/scan` + 静态 TF 为 858 条非局部边全部生成重叠率；双证据额外拒绝 11 条，ATE 1.1521 m；同时记录 naive 阈值敏感反例，默认关闭等待多序列验证 |
| DDS domain 上界护栏 | 避免 PID 取模生成 Fast DDS 无法映射端口的 domain | 修正连续语音/Nav2/OpenLORIS 等 6 个入口，并用仓库测试保证所有公式最大值不超过 232 |
| 系统就绪状态收敛 | 替代 launch/test 中分散的固定 sleep、topic graph 猜测和日志字符串判断 | 新增 `ComponentHealth`、`SystemReadiness`、心跳超时聚合器和 profile 化启动门禁；保留音频/仿真数据质量探针 |
| Agent 参数与 launch 契约收敛 | 消除 online/offline 节点、YAML、Gazebo/Nav2 launch 中重复默认值和转发映射 | 新增共享参数 schema、ROS range/enum 描述、启动前校验、只读快照与组合 launch 转发契约；provider YAML 仅保留模型配置 |
| Agent 并发运行时收敛 | 消除端点 timer、busy/worker 和多命令 NLU 入队的双份状态机 | 新增 `AsrEndpointRuntime`、`AgentExecutionRuntime` 和 `CommandEnqueueDecision`；统一异常隔离、busy 复位、batch metadata 与关闭时 timer/cancel 语义 |
| LLM 流式协议收敛 | 消除 online/offline 的 token parser、TTS 分句与动作选择双份实现 | 新增 `StreamingTurnRuntime/Result`；统一格式错误回退、确定性动作优先级、语义安全阻断和记忆动作口径，provider 仅保留 TTS/latency adapter |
| LoRA/Q8 证据收口 | 真实完成训练、量化和同口径基线对照，不再只提供 dry-run | 96 条确定性训练集与 43 条独立 holdout 无文本重叠；动作语义 30.23%→53.49%，严格总分仍为 25.58%；训练、提示词、GGUF 和报告由 SHA256 审计绑定 |
| 用户上下文一致性收敛 | 消除身份、画像 prompt、偏好和低置信度写保护的双份节点逻辑，并修复异步声纹切换竞态 | 新增 `UserContextRuntime/Snapshot`；命令入队时冻结身份/偏好，prompt、动作与 interaction 共用同一快照；预解析动作恢复归入 `AgentControlPlane` |
| Agent Lifecycle 资源治理 | 让 online/offline 的生命周期状态对应真实 provider、线程和 publisher，而非只保留进程级启停 | 两个 Agent 升级为 `LifecycleNode`；configure/activate/deactivate/cleanup/on_error 统一资源边界，managed publisher、协作取消、安全 STOP、STOPPED health、manager 依赖顺序和 cleanup 后重建均有自动验收 |
| Agent ROS I/O 契约收敛 | 消除 online/offline 节点重复接线、topic 字符串和 QoS 漂移 | 新增 `AgentRosIo`、不可变 `AgentTopicContract` 与 `audio_qos`；节点只注入 callback，PCM best-effort、控制 reliable、状态 latched，并在 inactive 关闭健康心跳 |
| Agent turn 指标强类型化 | 删除在线/离线双 topic 与 `String + JSON` 指标协议 | 新增 `AgentTurnMetrics` 和唯一 `metrics_transport.py`；统一 `/agent/metrics`，source 区分模式，NaN/三态 target 表达缺失值，监控与验收共享转换 Adapter |
| Agent Lifecycle 编排收敛 | 消除 online/offline 对 active/stopping、endpoint、execution 和安全停机顺序的双重所有权 | 新增组合式 `AgentLifecycleRuntime`；统一 bind/activate/deactivate/release/shutdown、priority STOP 与 quiescence 报告，provider 仅注入输入启停 hook |
| Agent 包依赖收敛 | 消除 offline 复用 online 内部业务模块和语音 sidecar 形成的反向依赖 | 新增 `embodied_agent_core` 与 `embodied_voice_frontend`；公共领域/编排/记忆/transport 和 VAD/KWS/声纹 Adapter 分别归位，online/offline 只保留各自 provider |
| Python/C++ QoS 语义对齐 | 删除 VAD/KWS/声纹节点手写 QoS 和跨语言命名漂移 | Python/C++ 统一 command/event/state/sensor/audio/diagnostics 六类 profile；KWS score 与 PCM 使用 best-effort，身份/健康使用 transient-local，控制和事件保持 reliable + volatile |
| C++ 运行时模块与 bridge 生命周期收敛 | 避免 control/audio/hardware 因单一库产生无关链接，并让调度器具备可管理启停语义 | 同一 ROS 包内拆为 3 个 CMake target；typed bridge 注册 component 并升级 Lifecycle，显式 callback group、inactive 拒绝、deactivate 取消清队列及 cleanup 后重建均有验收 |
| Agent 应用层与 turn 数据面收敛 | 删除 online/offline 重复的 transcript、记忆、队列、用户快照、动作批次和模型 turn 编排 | 新增组合式 `AgentApplicationRuntime` 与在线/离线 `*StreamingTurnRuntime`；主节点缩至 543/676 行，provider 差异通过 callback 注入且公开 ROS 契约不变 |
| 运行时证据口径收口 | 避免短时、fixture、fallback 后结果被误写成真实长稳或模型原始能力 | 在线/离线分别生成 5 分钟报告，增加 Agent 模式、queue reject、P50/P95 和统一事实汇总；缺失或失败证据明确标记，不阻塞无麦克风 CI |
| 离线 E2E Lifecycle 就绪探针 | 修复 Agent 已激活但 smoke 仍等待旧 `ready` 日志直至超时 | 复用 Lifecycle `GetState` 服务，以只读 wait policy 等待 active；Lifecycle manager 保持唯一转换者，当前真实模型 fixture 整轮 1428.6 ms |

## 2. 当前完成度结论

当前项目已经达到“语音输入 → 大模型/规则动作解析 → ROS 2 安全校验 → Gazebo 仿真控制”的主链路目标。

已具备的展示点：

- ROS 2 C++ 节点：音频前端、ActionGuard、typed action bridge、仿真执行层；动作控制面不再依赖 JSON 字符串解析。
- Python Agent：在线/离线 provider、连续语音会话、命令队列、LLM/TTS 编排。
- 工程化接口：自定义 msg/action、Lifecycle、BehaviorTree.CPP、pluginlib。
- 演示能力：真实麦克风连续语音、多动作序列、急停抢占、Gazebo 运动验证。
- 多命令能力：一条 ASR final 可被轻量 NLU 解析为多个队列项，并按 ROS 2 Action result 顺序执行。
- 导航演示能力：支持“去门口”“前往书桌”“依次去门口、书桌、起点”等语音目标点/多点巡航命令，并通过 typed Action 驱动仿真 executor、Nav2 action bridge 或完整 TurtleBot3/Nav2 bringup。
- SLAM/避障能力：受控漂移建图、回环优化、Ceres/GTSAM A/B、地图复用定位规划，以及基于速度预测的动态障碍 costmap 插件。
- 测试体系：单元测试、集成 smoke、Gazebo 验收、真实麦克风辅助统计。
- 汇报材料：已补充 15 分钟项目汇报与代码走读稿，便于按链路讲解关键文件和技术取舍。

需要谨慎表述的边界：

- 当前硬件控制是预留/mock，不是实体机器人完整验收。
- 当前已提供 TurtleBot3/Nav2、地图构建/复用和预测动态避障重型验收；真实 rosbag 回放报告和实体机器人仍是后续证据。
- 离线 LoRA 训练、合并、Q8 与 43 条合成 holdout 对照已复现；只能引用动作语义
  30.23%→53.49% 和严格总分 25.58% 等实测值，不外推为真实语音准确率。
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
- 固定 10 命令、5 分钟真实麦克风 benchmark，量化识别率、动作成功率、误触发、queue reject
  与延迟 P50/P95；online/offline 报告使用独立文件名。
  自动测试不再冒充真人长时间证据。
- 新增 `continuous-voice-evidence` 单终端入口，自动启停控制链路并显示倒计时；benchmark
  即使未达门槛也保证生成现场与汇总两份报告，避免 `set -e` 提前中断留证。
- 新增 `runtime-evidence-summary`，用 `proven/failed/missing` 汇总两种长稳证据；离线 LLM
  原始动作准确率与 fallback+安全后的系统有效率永久分栏。
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

### P3：真实数据与导航消融

- 已补齐 OpenLORIS ROS 1 bag 的 ROS 2 `/clock`/TF/LaserScan 流式 Adapter、轨迹 recorder、
  Ceres/GTSAM 公平 A/B、fixture 门禁和 `office1-1` 实验报告；当前 322 个对齐位姿、
  99.65% 覆盖率，Ceres/GTSAM ATE RMSE 为 2.879/2.890 cm。
- `office1-7` 在旧 10 秒宽松定义下有 2 次短时回访和 449 个对齐位姿，但 46 条图边全部相邻；
  六组参数与 C++ Karto trace 已把失败定位到 near-linked 候选排除。按正式 60 秒长回环门槛，
  它没有真值事件，因此不能再作为长回环召回率证据。
- 已用视觉联络表人工标注玻璃隔断与动态人员遮挡；画面不支持长走廊标签，已显式保留 negative
  evidence。
- 已完成真实前端 accepted-edge 阈值消融：把 1.43 GB 原包裁为带来源链的约 5 MB SLAM-only
  bag，固定数据/GTSAM/评估器比较 baseline、chain、response、search、combined 和 extreme 六组。
  每组 449 个匹配位姿、49～52 秒；46 条图边始终全部相邻，说明仅放宽公开参数仍未触发非局部
  约束。Karto 候选/拒绝 instrumentation 已完成；真值排名第一的 `market1-3` 因原始 bag 缺少
  `/scan` 被传感器契约拒绝，当前转向 `corridor1-1`（约 272.5 秒、220.1 米、2 次至少相隔
  60 秒的位置回访）验证真实候选与 accepted loop。
- `corridor1-1` 已完成 GTSAM 长序列证据：参考覆盖 99.9743%、正式运行 ATE RMSE 1.676 m；最终
  轨迹几何恢复 2/2 个位置回访，但 8 次 Karto 原生 closure 中只有 7 次落在真值覆盖内，其中
  1 条相对位姿残差达标，accepted-edge 长回访 recall 为 0。先行运行曾产生 4 次 closure，暴露
  异步前端运行间波动。node-id 间隔不再作为正式 loop 分类，
  改用原生 closure scan id 与 SE(2) 相对位姿残差。
- 已完成同一 `corridor1-1` 固定图的后端鲁棒核消融：图哈希、1834 节点、2751 约束和 1828 个
  真值匹配姿态在四组间完全一致；Cauchy non-local 指标最好。下一步转向前端感知混淆抑制，
  不再通过调整后端掩盖错误 closure 或回环漏检。
- 在同一固定图上新增无真值在线依赖的一致性门控消融：以优化前图预测计算创新量，2 m / π/4
  阈值拒绝 23 条明显异常非局部边，Cauchy + gate ATE 为 1.1713 m。该启发式可能在大漂移时
  误拒真回环，默认关闭，且不改变前端 precision 的正式评价口径。
- 已完成动态障碍 current-only、常速度、Kalman、IMM 同场景消融：C++ 固定输入报告预测
  RMSE/遮挡/停车过冲，四轮 Gazebo/Nav2 报告验证 lethal cost、重规划、到达和最终零速；场景、
  地图栅格和 Nav2 参数已纳入 SHA256 一致性门禁。输入仍是合成 `PoseArray`，物理动态 actor 与
  传感器遮挡属于后续增强。
- 继续细分 Nav2 planner/controller/behavior tree 失败原因和恢复行为指标。

### P3：可选增强

- 接入真实声学 KWS，例如 sherpa-onnx keyword spotting 或 openWakeWord。
- 接入 WebRTC AEC/NS，改善扬声器回声环境。
- 引入更接近 Nav2 的行为树 XML 和复杂安全策略。
- 接真实硬件底盘或串口设备，完成 UART/SPI 实体验收。

## 5. 不建议近期优先做的事情

- 在公开 rosbag 和现有 Nav2 证据未收口前继续堆复杂导航行为：会增加演示面，但不能回答真实漂移和退化问题。
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
