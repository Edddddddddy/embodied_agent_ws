# 语音到仿真、自动建图与导航：端到端代码走读

本文以当前源码为准，是“语音输入到仿真执行”的精确代码地图，供代码走读、故障定位和汇报使用。
它覆盖真实麦克风 → 音频/VAD/ASR → 会话/队列/NLU/LLM → typed `RobotCommand` → C++
ActionGuard/Scheduler → BehaviorTree/pluginlib → Gazebo，以及自动 frontier SLAM → 存图 →
AMCL/Nav2 → 语义巡航 → 日志证据。路径均相对于 `/home/ubuntu/embodied_agent_ws`。

## 1. 先理解两条链，而不是把所有逻辑画成一条线

### 1.1 普通动作与导航链

```text
麦克风
 -> AudioFrontendNode
 -> QwenRealtimeAsr / SherpaZipformerAsr
 -> OnlineAgentNode / OfflineAgentNode
 -> AgentApplicationRuntime
 -> AgentControlPlane + ContinuousCommandQueue + CommandNLU
 -> /agent/action_candidate (RobotCommand)
 -> ActionGuardNode
 -> /robot/action_command_typed (RobotCommand)
 -> TypedActionBridgeNode + ActionScheduler
 -> /robot/execute_command (ExecuteRobotCommand Action)
 -> SimulationControlNode
 -> CommandBehaviorTree
 -> GazeboRobotExecutor / Nav2RobotExecutor
 -> /cmd_vel / Nav2 Action
```

### 1.2 自动建图与导航任务链

```text
/agent/asr_final
 -> SessionOrchestratorNode._on_asr_final
 -> parse_session_command
 -> RUN_AUTOMATIC_MISSION
 -> _run_automatic_mission
 -> Explore Lite frontier 探索
 -> /map + /explore/status 完成判定
 -> nav2_map_server map_saver_cli
 -> 停 mapping graph
 -> 以保存地图启动 AMCL/Nav2
 -> 检查 Nav2 Action + Lifecycle ACTIVE
 -> “去入口”“依次去厨房、办公室”回注 /agent/text_input
 -> 复用普通动作与导航链
```

必须讲清的边界：

- 编排器启动后先自动拉起 mapping graph；用户说“开始自动巡检建图”后才启动 frontier 自主探索。
- 高层建图事务不属于 `CommandNLU` 的底盘动作，它由 `ManageSlamSession`/`SlamSessionState` 表达。
- 自动巡航重新经过 Agent、typed msg、ActionGuard、Action、BT 和 Nav2 executor，没有直连 `/cmd_vel`。
- 该办公演示默认使用 Nav2 官方 TurtleBot3 bringup 的 SLAM Toolbox 路径；仓库中的 GTSAM/自研回环实验不是此演示的默认必经链。

## 2. 全链路分层索引

| 层 | 关键文件 | 类/函数 | 输入 | 输出 |
| --- | --- | --- | --- | --- |
| 部署 | `scripts/voice_slam_nav_showcase.sh` | `run_voice_stage()` | auto/mapping/save/navigation | 阶段 ROS graph |
| 麦克风 | `audio_frontend_node.cpp` | `AudioFrontendNode::input_callback()` | PortAudio PCM | `/audio/clean_pcm` |
| DSP/VAD | `audio_processing.cpp` | `NlmsAudioEnhancer::process()`、`SpeechEndpointDetector::update()` | PCM frame | endpoint/metrics |
| 在线 ASR | `providers/qwen_asr.py` | `QwenRealtimeAsr.push_audio()/commit()` | clean PCM | partial/final |
| 离线 ASR | `providers/sherpa_asr.py` | `SherpaZipformerAsr.push_audio()/commit()` | clean PCM | partial/final |
| final 稳定 | `transcript_stabilizer.py` | `finalize()` | partial + final | 稳定 transcript |
| 会话 | `continuous_voice.py` | `ContinuousVoiceSession.accept()` | transcript | command/reject/priority |
| 控制面 | `agent_control_plane.py` | `accept_transcript()`、`enqueue_command()` | command | batch/queue decision |
| NLU | `command_nlu.py` | `CommandNLU.parse()` | 中文命令 | `ActionCommand[]` |
| LLM | `streaming_turn.py` | `StreamingTurnRuntime.feed()/finish()` | token stream | speech + actions |
| 队列 worker | `agent_execution_runtime.py` | `_run_worker()` | queued command | execution events |
| typed seam | `ros_action_transport.py` | `action_command_to_message()` | `ActionCommand` | `RobotCommand` |
| C++ 安全 | `action_guard_node.cpp` | `on_candidate()` | candidate | guarded command/reject |
| C++ 调度 | `action_scheduler.cpp` | `enqueue()/complete()` | guarded command/result | dispatch/cancel/result |
| Action bridge | `typed_action_bridge_node.cpp` | `dispatch_goal()` | scheduler event | ROS 2 Action goal |
| Action server | `simulation_control_node.cpp` | `handle_accepted()` | `ExecuteRobotCommand` | executor/feedback/result |
| BT | `command_behavior_tree.cpp` | `start()/tick()/cancel()` | command+safety+state | BT outcome |
| Gazebo | `gazebo_robot_executor.cpp` | `execute()/step()` | motion command/scan | `/cmd_vel` |
| 自动任务 | `showcase_session_node.py` | `_run_automatic_mission()` | system intent | explore/save/Nav2 |
| frontier | 同上 + Explore Lite | `_wait_for_frontier_completion()` | map/status | completion reason |
| 存图 | 同上 | `StageProcessManager.save_map()` | `/map` | YAML + PGM |
| Nav2 | `nav2_robot_executor.cpp` | `send_navigate_goal()/send_follow_goal()` | semantic target | Nav2 result |
| 现场日志 | `continuous_voice_monitor.py` | `ContinuousVoiceMonitor` | typed topics | `[audio]...[result]` |
| 自动证据 | `test_voice_slam_session_orchestrator.py` | `SessionProbe` | ROS graph | report JSON |

## 3. 启动与阶段进程

### 3.1 入口

| 内容 | 位置 |
| --- | --- |
| 统一入口 | `scripts/acceptance_test.sh` |
| 演示 mode | `voice-slam-workplace-demo {offline\|online}` |
| 自动脚本 | `scripts/voice_slam_nav_showcase.sh auto {offline\|online}` |
| 连续 Nav2 runner | `scripts/continuous_nav2_voice_control.sh` |
| launch | `src/embodied_simulation/launch/voice_nav2_turtlebot3.launch.py` |

调用顺序：

1. `acceptance_test.sh` 调用 `voice_slam_nav_showcase.sh auto`。
2. `auto` 分支 source 环境，运行 `voice_slam_session_orchestrator`。
3. `SessionOrchestratorNode._worker_loop()` 先调用 `_start_mapping()`。
4. `_start_mapping()` → `StageProcessManager.start("mapping")`。
5. `StageProcessManager.command()` 生成 `voice_slam_nav_showcase.sh mapping <mode>`。
6. mapping 分支设置 world、SLAM、executor、places，再进入 `continuous_nav2_voice_control.sh`。
7. runner 组装参数并 launch `voice_nav2_turtlebot3.launch.py`。

为什么按阶段重启而不是在一个 graph 内热切 SLAM/AMCL：SLAM Toolbox 与 AMCL 对 map/TF 的所有权不同；彻底关闭进程组能避免旧 Explore Lite、component 或 lifecycle 状态残留。代价是阶段切换慢于热切换，但演示和失败恢复更确定。

失败语义：mapping/navigation 子进程意外退出由 `_check_child_process()` 置为 `FAILED`；新阶段没有产生新 readiness generation 时，`_wait_for_new_ready()` 超时；模型、Nav2 或环境缺失由 preflight/启动脚本提前失败。

上游：用户验收命令。下游：Gazebo、Agent、SLAM/AMCL/Nav2 和执行层 ROS graph。

## 4. C++ 麦克风、AEC 与端点

### 4.1 文件和接口

| 内容 | 位置 |
| --- | --- |
| 节点 | `src/embodied_agent_cpp/src/audio_frontend_node.cpp` |
| 主类 | `embodied_agent_cpp::AudioFrontendNode` |
| DSP 接口 | `include/embodied_agent_cpp/audio_processing.hpp` |
| DSP 实现 | `src/embodied_agent_cpp/src/audio_processing.cpp` |
| 核心函数 | `input_callback()`、`processing_loop()`、`playback_loop()` |
| 核心类 | `NlmsEchoCanceller`、`NlmsAudioEnhancer`、`SpeechEndpointDetector` |

输入是 PortAudio 默认单声道 PCM16，通常 16 kHz、20 ms 一帧；`/audio/tts_pcm` 同时是扬声器播放流和 NLMS 的参考流。

处理顺序：

1. `input_callback()` 仅复制 PCM 到有界 `input_queue_`，不做模型、DDS 或日志。
2. 队列满时丢最旧帧并增加 `dropped_input_frames_`，不允许无界内存增长。
3. `processing_loop()` 调用 `audio_enhancer_->process(frame)`。
4. AEC 启用时 `NlmsAudioEnhancer` 调 `NlmsEchoCanceller::process()`。
5. `compute_audio_frame_metrics()` 计算 RMS、peak、speech。
6. `SpeechEndpointDetector::update()` 聚合 speech/silence，产生 utterance start/end。
7. `playback_loop()` 先 `add_reference()` 再写声卡，保持 AEC 参考与实际播放一致。

| 输出 | 类型 | 作用 |
| --- | --- | --- |
| `/audio/clean_pcm` | `UInt8MultiArray` | 清理后 PCM16 |
| `/audio/speech_started` | `Empty` | utterance 开始 |
| `/audio/speech_ended` | `Empty` | utterance 结束 |
| `/audio/silence_timeout` | `Empty` | 兼容 commit 触发 |
| `/audio/frontend_metrics` | `AudioFrontendStatus` | RMS/peak/speech/丢帧/enhancer |
| `system/component_health` | `ComponentHealth` | 前端健康 |

失败语义：PortAudio 初始化失败使进程 fatal；消费者过慢丢旧帧并上报；短于 `min_utterance_ms` 的噪声不产生有效结束；超过 `max_utterance_s` 强制结束。当前 NLMS AEC 可用，但 NS/AGC 只记录 requested、active=false，不能描述为已完成 WebRTC 增强。

设计原因：声卡 callback 必须极短；PCM 使用 best-effort sensor QoS，拥塞时丢旧帧优于积压旧语音；VAD 与 endpoint 分开，换 Silero/WebRTC 不改 Agent 接口。上游是 WSLg/PulseAudio， 下游是 ASR 与可选 sidecar。

## 5. VAD provider 与 ASR commit

| 内容 | 位置 |
| --- | --- |
| launch seam | `embodied_agent_bringup/voice_frontend_launch_contract.py::voice_frontend_nodes()` |
| Silero | `embodied_voice_frontend/silero_vad_node.py` |
| WebRTC | `embodied_voice_frontend/webrtc_vad_node.py` |
| sidecar 状态机 | `silero_vad_sidecar.py::StreamingVadEndpoint` |
| commit 运行时 | `embodied_agent_core/asr_endpoint_runtime.py::AsrEndpointRuntime` |

`vad_provider=energy` 时，C++ 前端拥有 endpoint；`silero/webrtc` 时 launch 关闭其 `endpoint_events_enabled`，由外部节点订阅 clean PCM 并发布同协议的 start/end。这样不会出现两个 VAD 同时提交。

Agent `_on_speech_ended()` 和兼容的 `_on_silence_timeout()` 都调用 `_commit_asr_endpoint()`；`AsrEndpointRuntime.request()` 用短去重窗口、generation 和 timer 把它们收敛为至多一次 commit。`asr_commit_delay_ms` 给尾部 PCM/partial 留出稳定时间，减少“左转九十度”只 final 成“左转”。

在线 commit 直接调用 `QwenRealtimeAsr.commit()`；离线 commit 被放入 `_asr_events`，确保 Sherpa audio 与 `input_finished()` 在同一 worker 线程。

失败语义：busy/inactive/重复 endpoint 返回 false；Lifecycle 停用使 generation 失效并取消 timer；provider commit 异常经 `on_error` 输出，不能从 timer 线程静默消失。

上游：energy/Silero/WebRTC endpoint。下游：在线 WebSocket 或离线 Sherpa stream finalization。

## 6. 在线与离线 ASR

### 6.1 在线 Qwen ASR

| 内容 | 位置 |
| --- | --- |
| 节点 | `src/embodied_online_agent/embodied_online_agent/online_agent_node.py` |
| adapter | `providers/qwen_asr.py::QwenRealtimeAsr` |
| 音频入口 | `OnlineAgentNode._on_clean_audio()` |
| partial/final | `_on_asr_partial()`、`_on_asr_final()` |

`_on_clean_audio()` → `push_audio()` → PCM base64 → DashScope WebSocket。SDK transcription text 事件回调 partial，completed 事件回调 final。缺 `DASHSCOPE_API_KEY` 或 SDK 会在 start/configure 阶段明确失败；inactive 时不消费音频。

### 6.2 离线 Sherpa ZipFormer

| 内容 | 位置 |
| --- | --- |
| 节点 | `src/embodied_offline_agent/embodied_offline_agent/offline_agent_node.py` |
| adapter | `providers/sherpa_asr.py::SherpaZipformerAsr` |
| ROS 回调 | `OfflineAgentNode._on_audio()` |
| 串行 worker | `OfflineAgentNode._run_asr()` |

调用链：`_on_audio()` 把 `("audio", pcm)` 入队；worker 调 `push_audio()`；adapter 将 PCM16 转 float32，调用 `accept_waveform()` 与 `_decode_ready()`；文本变化发 partial；commit 调 `input_finished()`、decode、final，再创建新 stream。

Sherpa stream 有状态，所以 audio/commit 必须单线程；ROS callback 只入队，避免同步 decode 阻塞 executor。队列满时丢最旧音频并报警，commit 尽量保留。模型或 native wheel 缺失使 configure 失败；单次 decode 异常记录后 worker 继续；deactivate 插入 stop、join、清残留，避免跨会话拼句。

两条链共同上游是 `/audio/clean_pcm`，共同下游是节点 `_on_asr_partial()`/`_on_asr_final()`。

## 7. final 稳定、公开分叉与会话门控

| 内容 | 位置 |
| --- | --- |
| final 稳定 | `embodied_agent_core/transcript_stabilizer.py::TranscriptStabilizer` |
| ROS I/O | `agent_ros_io.py::AgentRosIo` |
| topic 契约 | `ros_topics.py::AgentTopicContract` |
| 会话 | `continuous_voice.py::ContinuousVoiceSession` |
| 控制面 | `agent_control_plane.py::AgentControlPlane.accept_transcript()` |
| 应用入口 | `agent_application_runtime.py::AgentApplicationRuntime.accept_transcript()` |

在线/离线 `_on_asr_final()` 的共同顺序：

1. 检查 Lifecycle active；非连续模式 busy 时抑制重叠 final。
2. `TranscriptStabilizer.finalize(text)` 只从近期 partial 恢复安全尾部槽位。
3. 如恢复，发布 `RecognitionFeedback`。
4. `AgentRosIo.publish_asr_final()` 发布 `/agent/asr_final`。
5. 调用 `AgentApplicationRuntime.accept_transcript()`。

`/agent/asr_final` 是公开事件，产生两个下游：Agent 控制面和 SLAM `SessionOrchestratorNode`。这就是普通动作与高层任务的分叉点。

控制面顺序：归一化 → `ContinuousVoiceSession.accept()` → filler/duplicate/wake/sleep/timeout → `CommandCompleter.complete()` → priority stop/navigation cancel → command。一次“小智”唤醒后可连续控制；“左转/前进”只做安全默认补全；停下/急停不补全、不去重，直接清队列、取消当前序列并发布 priority STOP。

| 输出 topic | 含义 |
| --- | --- |
| `/agent/session_state` | awake/sleeping |
| `/agent/wake_event` | wake/rejected/sleep |
| `/agent/recognition_feedback` | normalized/completed/retry/ignored |
| `/agent/state` | listening/thinking/queued/error |

失败语义：filler/duplicate 显式 ignored；session timeout 提示重新唤醒；speech start 会 clear 旧 partial；inactive final 不下发。会话放在 provider 之外，保证在线/离线和外部 KWS 使用同一业务状态机。

## 8. NLU、多命令、LLM 与 TTS

### 8.1 本地轻量 NLU

| 内容 | 位置 |
| --- | --- |
| NLU | `embodied_agent_core/command_nlu.py::CommandNLU` |
| 入口 | `CommandNLU.parse()` |
| 小模型 | `CharacterNgramIntentModel` |
| 槽位 | `_motion_action_and_slots()`、`_turn_action_and_slots()` |
| fallback | `command_fallback.py::parse_fallback_actions()` |

`parse()` 先阻断疑问、否定、越权高速和危险组合；`_find_anchors()` 再按 move/turn/stop/navigation 等锚点划分 segment，不只依赖标点；`CharacterNgramIntentModel.predict()` 对字符 bigram 原型做余弦相似度；槽位函数抽取方向、速度、距离、角度、时长、目标和 waypoints。

```text
向右转，然后向前走一秒
 -> batch 1/2: turn_right -> turn
 -> batch 2/2: move_forward -> move
```

`走正方形` 展开为 move/turn primitives；超过单条 10 秒的长距离会拆段。句中出现 stop 只保留 priority STOP；“九十度”却没有方向返回 `missing_turn_direction`；灯缺颜色返回 `missing_led_color`；无锚点/低置信度进入 LLM/fallback，不盲猜动作。

### 8.2 统一流式 turn

| 内容 | 在线 | 离线 |
| --- | --- | --- |
| turn | `online_turn_runtime.py::OnlineStreamingTurnRuntime` | `offline_turn_runtime.py::OfflineStreamingTurnRuntime` |
| LLM | `OpenAiCompatibleLlm.stream()` | `LlamaCppLlm.stream()` |
| TTS | `QwenRealtimeTts.synthesize()` | Sherpa/Summer + `PseudoStreamingTtsPipeline` |
| 共用协议 | `streaming_turn.py::StreamingTurnRuntime` | 同左 |

`StreamingTurnRuntime.feed()` 增量解析 `TaggedStreamParser` 的 `<speech>/<action>`；speech 可立即切句送 TTS，action 必须等完整闭标签和合法 payload。`finish()` 的动作选择顺序是：确定性 fallback → 危险语义阻断 → 模型动作。

在线 turn 用线程让 LLM token 与持久 Qwen TTS session 并行。离线 `LlamaCppLlm` 访问 llama-server OpenAI-compatible stream；已经发出 token 后不静默重试，避免拼接两段回复；`PseudoStreamingTtsPipeline` 用 message/audio 双缓冲并行 LLM、整句合成和 PCM 发布。

上游是 queued/direct text；输出是 response/TTS 与 `ActionCommand[]`。协议损坏时说出格式错误提示，但不从半截 action 猜控制命令。

## 9. 批次队列与执行线程

| 内容 | 位置 |
| --- | --- |
| FIFO | `continuous_voice.py::ContinuousCommandQueue` |
| 入队 | `AgentControlPlane.enqueue_command()` |
| worker | `agent_execution_runtime.py::AgentExecutionRuntime._run_worker()` |
| 恢复上下文 | `AgentApplicationRuntime.run_queued_turn()` |
| 动作批次 | `action_sequence.py::SequentialActionPublisher` |

NLU 多命令共享 `batch_id`，每项保存 `batch_index/batch_size/source_text/nlu_intent/nlu_confidence/preparsed_actions`，以及入队时冻结的用户上下文和离线 latency。队列有固定容量和 TTL：满时 `queue_full`，旧意图取出时发 `expired` 而不是延迟执行。

worker 顺序：get → `execution_started` → `run_queued_turn()` → 有预解析动作则 `_run_preparsed_turn()`，否则 model turn → `execution_finished` → finally 复位 busy。单条 provider 异常不会杀死长时间 worker。

`SequentialActionPublisher.publish()` 给批内每个动作分配唯一 request ID，再一次性发布给 C++ scheduler。Python 只等待语义批次是否完成；真正决定下一 goal dispatch 时机的是 C++ `ActionScheduler`，因此没有双重调度。组合/连续动作按 ID 等 result，超时或失败时发布 priority STOP。

输出：`/agent/command_queue`、`/agent/command_execution`、`/agent/nlu_parse`。Lifecycle 停用使用 `AgentExecutionCancelled`，属于受控取消，不误报 provider 故障。

## 10. typed RobotCommand 与 C++ ActionGuard

### 10.1 typed seam

| 内容 | 位置 |
| --- | --- |
| 领域动作 | `embodied_agent_core/types.py::ActionCommand` |
| 转换 | `ros_action_transport.py::action_command_to_message()` |
| 消息 | `src/embodied_agent_interfaces/msg/RobotCommand.msg` |
| 候选发布 | 两个 Agent 的 `_publish_action_candidate()` |

`RobotCommand` 携带 `command_id/source/priority/action_type`，以及 motion、accessory、navigation 的强类型字段。`ARC` 在候选层保留，ActionGuard 通过后规范化成带 linear_x+angular_z 的 MOVE。

控制 topic 不使用 JSON：typed msg 提供编译期字段/枚举检查，并适配 C++、DDS、rosbag 和 QoS。JSON 只用于报告、模型协议和人类日志。未知领域动作在转换 seam 抛 `ValueError`，不产生半合法消息。

### 10.2 ActionGuard

| 内容 | 位置 |
| --- | --- |
| 节点 | `src/embodied_agent_cpp/src/action_guard_node.cpp::ActionGuardNode` |
| 回调 | `on_candidate()` |
| 校验 | `action_validator.cpp::ActionValidator::validate()` |
| 缓冲 | `GuardedCommandOutbox` |
| 输出 | `/robot/action_command_typed`、`/robot/action_rejected` |

校验包括：action 白名单；priority 仅 STOP/CANCEL；无关字段为空；数值 finite；MOVE/ARC 线速度 `[-0.5,0.5]`、角速度 `[-1.5,1.5]`、时长 `[0,10]`；TURN 禁止 linear_x；LED/mode/place 白名单；waypoint 最多 8 个、loops 1..3。

合法命令先进入 outbox；`flush_outbox()` 检查下游 subscription，短时未发现 scheduler 就缓冲，超时发布 `action_downstream_unavailable:<id>`。非法参数、buffer full、下游不可用都有明确 reject；inactive 时不把候选留到下一次 activate。

为什么 C++ 再校验：Python NLU/LLM 都是不可信上游；安全规则必须在执行边界唯一且不随 provider 替换。上游 `/agent/action_candidate`，下游 C++ Action bridge。

## 11. C++ Scheduler 与 ROS 2 Action bridge

| 内容 | 位置 |
| --- | --- |
| 纯调度器 | `src/embodied_agent_cpp/src/action_scheduler.cpp::ActionScheduler` |
| bridge | `typed_action_bridge_node.cpp::TypedActionBridgeNode` |
| Action | `src/embodied_agent_interfaces/action/ExecuteRobotCommand.action` |
| Action 名 | `/robot/execute_command` |

调用链：

1. `TypedActionBridgeNode::on_command()` 收 guarded command。
2. `ActionScheduler::enqueue()` 产生 dispatch/cancel/result/rejected 事件。
3. `process_events()` 适配为 ROS 操作。
4. `dispatch_goal()` 等 Action server，调用 `async_send_goal()`。
5. feedback callback 发布 `/robot/action_feedback`。
6. result callback → `complete_active()` → `ActionScheduler::complete()`。
7. 只有 result ID 等于 active ID 才推进下一项；迟到旧结果被忽略。
8. `/robot/action_result` 回到 Agent，闭合 `SequentialActionPublisher` 等待。

priority STOP/CANCEL 会给 pending 命令产生 canceled result、放到队首并取消 active；等 active 返回终态后才 dispatch priority，保持同一时刻一个 goal。`on_cancel_watchdog()` 把丢失的取消终态收敛为 `STATUS_TIMED_OUT/cancel_result_timeout`。

失败语义：缺/重复 ID 被拒绝；queue 满返回 `scheduler_queue_full`；server 不可用返回 `action_server_unavailable`；goal rejected 明确上报。重复 ID 不发布同 ID 假 result，避免原请求误消费。

Action 而不是 service/topic，是因为 move、turn、Nav2 都需要 feedback、取消、超时和终态。上游 `/robot/action_command_typed`，下游 `SimulationControlNode`。

## 12. Action server、BT、pluginlib 与 Gazebo

### 12.1 Action server

| 内容 | 位置 |
| --- | --- |
| 节点 | `src/embodied_simulation/src/simulation_control_node.cpp::SimulationControlNode` |
| 回调 | `handle_goal()`、`handle_accepted()`、`handle_cancel()` |
| 长动作 | `ActiveActionRuntime`、`ActionExecution` |
| ROS 输出 | `SimulationRosIo` |

STOP/CANCEL/SET_MODE/WAVE/SET_LED 是即时 Action；MOVE/TURN/NAVIGATE_TO/FOLLOW_WAYPOINTS 是长 Action。`handle_accepted()` 先启动 BT，再调用 executor；`control_tick()` 周期调用 executor `step()` 和 `update_active_action()`。`ActionExecution::update()` 按 cancel → blocked → timeout → duration 决定状态；Nav2 外部 result 可驱动终态，但本地 hard timeout 最高优先。

| 输出 | 作用 |
| --- | --- |
| `/cmd_vel` | Gazebo 差速速度 |
| `/robot/action_ack` | executor 接受/终态 |
| `/robot/bt_status` | stage/outcome/detail |
| `/robot/simulation_state` | scan 安全、模式、速度、原因 |
| `/diagnostics` | executor/scheduler 快照 |
| `system/component_health` | 组件 readiness |

inactive goal、executor reject、雷达制动、timeout、cancel 均产生明确终态；deactivate 先取消 active、stop executor、发布零速度，再停 managed publisher。

### 12.2 BehaviorTree.CPP

| 内容 | 位置 |
| --- | --- |
| XML | `src/embodied_simulation/config/command_tree.xml` |
| C++ | `src/embodied_simulation/src/command_behavior_tree.cpp` |
| 节点 | `ValidateCommandNode`、`CheckSafetyNode`、`ExecuteCommandNode`、`ConfirmResultNode` |

```text
ReactiveSequence:
ValidateCommand -> CheckSafety -> ExecuteCommand -> ConfirmResult
```

`CommandBehaviorTree::start()` 为每个 goal 建独立 blackboard/tree；`tick()` 每控制周期一次；`cancel()` halt tree。validate 防御直接 Action 调试入口，safety 映射 blocked，只有 succeeded 通过 confirm。BT 保留稳定编排，速度/Nav2 算法留在可单测 C++ 模块；相比巨型 XML 更易调试，相比硬编码 if/else 更可观测。

### 12.3 pluginlib 与 Gazebo 安全

| 内容 | 位置 |
| --- | --- |
| 抽象 | `include/embodied_simulation/robot_executor.hpp::RobotExecutor` |
| 清单 | `src/embodied_simulation/robot_executor_plugins.xml` |
| Gazebo | `src/embodied_simulation/src/gazebo_robot_executor.cpp` |
| 控制器 | `src/embodied_simulation/src/simulation_controller.cpp` |

`SimulationControlNode::on_configure()` 用 `ClassLoader<RobotExecutor>::createSharedInstance()` 装载 mock/Gazebo/Nav2 后端。Gazebo MOVE 同时接受 linear_x/angular_z，TURN 令 linear_x=0；WAVE/LED 在无附件模型时停止底盘并 ACK。

`SimulationController::update_scan()` 计算前/左/右扇区最小距离；`step()` 实现 manual timeout、scan stale 禁止自主前进、front emergency 制动、自动避障、右墙 PID 和速度斜坡。失败原因是 `scan_timeout/front_emergency` 等，Action 变为 blocked；stop/完成后速度归零。

注意：Gazebo executor 的 navigate/follow 只是无 Nav2 profile 的可观测替代运动；真实目标点规划必须切到 `Nav2RobotExecutor`。pluginlib 的价值是后端替换时不改上游 Action、BT 和 diagnostics。

## 13. 自动 SLAM 会话状态机

| 内容 | 位置 |
| --- | --- |
| 领域状态机 | `src/embodied_slam_tools/embodied_slam_tools/showcase_session.py` |
| ROS 编排器 | `showcase_session_node.py::SessionOrchestratorNode` |
| 高层 Action | `src/embodied_agent_interfaces/action/ManageSlamSession.action` |
| 状态 | `SlamSessionState.msg` |
| 任务配置 | `src/embodied_simulation/config/showcase_workplace_mission.yaml` |

`parse_session_command()` 只识别保存地图、开始导航、保存并导航、停止会话、自动巡检建图。普通 move/turn/目标点语句返回 None，仍由 Agent 处理。“开始自动巡检建图”映射为 `RUN_AUTOMATIC_MISSION`。

入口有两种：`_on_asr_final()` 消费 `/agent/asr_final`；`/slam/manage_session` 接收 `ManageSlamSession` goal。二者都生成 `CommandRequest`，进有界 `_requests`，由 `_worker_loop()` 串行执行。`ShowcaseSessionStateMachine.validate()` 禁止非法阶段跳转；`/slam/session_state` transient-local 保存最新快照，Action feedback 补足快速中间阶段。

高层任务 busy 时，急停由 `_on_asr_final()` 旁路普通 request queue，直接将 active request 标记 canceled；`_cancel_automatic_motion()` 同时发布“停下”和停止 explorer。重复系统意图 3 秒内去重；queue 满、非法阶段、busy 都有明确 reason。自动任务异常恢复到当前可用 MAPPING/NAVIGATING，而不是一律杀死会话。

## 14. mapping 与 Explore Lite frontier

### 14.1 mapping 启动

```text
SessionOrchestratorNode._worker_loop
 -> _start_mapping
 -> StageProcessManager.start("mapping")
 -> voice_slam_nav_showcase.sh mapping
 -> continuous_nav2_voice_control.sh
 -> voice_nav2_turtlebot3.launch.py
 -> nav2_bringup/tb3_simulation_launch.py (slam=True)
```

mapping profile 使用 `showcase_apartment.sdf.xacro`、`GazeboRobotExecutor`、`showcase_mapping_places.yaml` 和 `NAV2_SLAM=true`。SLAM Toolbox 发布 `/map` 与 `map->odom`；Gazebo 发布 `/scan`、`/odom`；没有 AMCL，所以不发 `/initialpose`。`prepare_frontier_nav2_params.py` 为探索阶段生成更严格的目标容差，避免近 frontier 未移动就判成功。

`src/embodied_slam/launch/mapping_baseline.launch.py` 是漂移、回环、Ceres/GTSAM 对比入口，不是此自动演示的默认启动路径；自研 loop frontend 默认 shadow，只有显式 flag 才 commit。

### 14.2 frontier 自动探索

| 内容 | 位置 |
| --- | --- |
| 安装 | `scripts/setup_frontier_exploration.sh` |
| 参数 | `src/embodied_simulation/config/frontier_exploration.yaml` |
| 角落脱困路线 | `showcase_workplace_mission.yaml::automatic_exploration.bootstrap_route` |
| 启动 | `StageProcessManager.start_explorer()` |
| map/status | `_on_map()`、`_on_explore_status()` |
| 等待 | `_wait_for_frontier_completion()` |
| 判定 | `showcase_session.py::exploration_completion_reason()` |

`_run_automatic_mission()` 在 mapping ready 后先调用
`parse_mapping_bootstrap_route()`，并通过 `_run_agent_text_action()` 依次执行 move/turn-only 路线，
让机器人从左下充电角驶入中央门洞。每一步仍经过 Agent→ActionGuard→ROS 2 Action；随后才执行：

```text
ros2 run explore_lite explore --ros-args --params-file frontier_exploration.yaml
```

Explore Lite 从 free/unknown 边界选 frontier，通过 Nav2 在建图中持续派发目标；移动产生新激光观测，SLAM 更新地图，再选下一目标。编排器同时观察 ExploreStatus、known cells、occupied cells、最后显著增长时间、最短运行时间和总超时。

覆盖阈值达标后，`exploration_completion_reason()` 返回 `no_frontiers`、`coverage_plateau` 或
`time_budget_coverage`。plateau 解决家具背后/墙外不可达 frontier 导致永久恢复的问题；时间预算
只在覆盖已达标时收口，覆盖不足仍失败。

失败语义：Explore Lite 未安装给出 setup 提示；进程提前退出报 code；status 结束但覆盖不足分别报 known/occupied；总超时报 timeout；取消时停止 explorer。上游是 `/map`、`/explore/status` 和 Nav2，输出是可审计完成原因与覆盖合格地图。

## 15. 存图、AMCL/Nav2 与自动巡航

### 15.1 保存和阶段切换

```text
_save_map
 -> StageProcessManager.save_map
 -> voice_slam_nav_showcase.sh save
 -> nav2_map_server map_saver_cli
 -> voice_built_map.yaml + voice_built_map.pgm
```

`save_map()` 检查返回码，并验证 YAML/PGM 都存在；成功转 `MAP_SAVED` 并保存路径。失败回到 MAPPING，使用户可修正后重试。

```text
_start_navigation
 -> StageProcessManager.stop (结束 mapping 进程组)
 -> StageProcessManager.start("navigation")
 -> voice_slam_nav_showcase.sh navigation
 -> continuous_nav2_voice_control.sh
 -> voice_nav2_turtlebot3.launch.py (slam=False, saved map)
```

navigation profile 启动 map server、AMCL、Nav2 和 `Nav2RobotExecutor`。Gazebo 出生位姿不变；保存地图以建图起点为 map 原点，故 `publish_nav2_initial_pose.py` 向 AMCL 发布 `(0,0,0)`。`_wait_for_new_ready()` 要求新 generation，防止误用旧 mapping READY。

`_wait_for_navigation_action_servers()` 还检查 `/navigate_to_pose`、`/follow_waypoints` ready，以及 bt_navigator/waypoint_follower Lifecycle ACTIVE，避免“总体 readiness 绿了但长任务 server 不能用”。

### 15.2 Nav2 executor

| 内容 | 位置 |
| --- | --- |
| plugin | `src/embodied_simulation/src/nav2_robot_executor.cpp::Nav2RobotExecutor` |
| 地点 | `src/embodied_simulation/src/nav2_places.cpp` |
| 单点 | `send_navigate_goal()` |
| 多点 | `send_follow_goal()` |
| 多点终态策略 | `src/embodied_simulation/src/nav2_result_policy.cpp::evaluate_follow_waypoints_result()` |
| 配置 | `src/embodied_simulation/config/showcase_mapping_places.yaml` |

`configure()` 从 `EMBODIED_NAV2_PLACES_FILE` 加载语义地点，创建 NavigateToPose/FollowWaypoints clients 和内部 executor。`Nav2Places::to_pose_stamped()` 把 target 转 map pose；结果映射为 `ActionExecutionState` 与保留 target/error 的 detail。`evaluate_follow_waypoints_result()` 还要求 `error_code=0` 且 `missed_waypoints` 为空，防止 WaypointFollower 在部分目标失败后仍返回协议 `SUCCEEDED` 而被误判为业务成功。

generation 防止 stop 后迟到的旧 callback 覆盖新目标；若旧 goal response 在 stop 后才到，还会主动取消，避免“幽灵导航”。未知地点、server unavailable、goal reject/timeout、canceled/aborted 都形成可观测 blocked/canceled detail。

### 15.3 自动任务为何没有绕过 Agent

任务 YAML 给出 `navigate_text: 去入口`、`patrol_text: 依次去厨房、办公室`。`_run_agent_text_action()` 发布到 `/agent/text_input`，等待期望 `/agent/action_candidate`，记录 command_id，再等待相同 ID 的 `/robot/action_result` 成功；单点成功后才开始多点巡航。

因此最终调用仍是：Agent NLU → RobotCommand → ActionGuard → Scheduler → ExecuteRobotCommand → BT → Nav2RobotExecutor → NavigateToPose/FollowWaypoints → Nav2 Planner/Controller → `/cmd_vel`。

## 16. 日志、证据与失败定位

### 16.1 现场 monitor

| 内容 | 位置 |
| --- | --- |
| monitor | `scripts/continuous_voice_monitor.py::ContinuousVoiceMonitor` |
| 统计 | `MonitorStats.format_summary()` |
| 样本 | `AsrNluSampleRecorder` |
| readiness | `scripts/system_readiness_check.py` |

关键回调/输出：`_on_audio()` → `[audio]`；`_on_asr()` → `[asr]`；`_on_queue()` → `[queue]`；`_on_execution()` → `[exec]`；`_on_action()` → `[action]`；`_on_action_feedback()` → `[feedback]`；`_on_action_result()` → `[result]`；`_on_action_ack()` → ACK。monitor 读取 typed msg 后仅为终端展示转成字典，控制链本身不是 JSON。

### 16.2 自动证据

| 内容 | 位置 |
| --- | --- |
| 重型门禁 | `scripts/smoke_test_voice_slam_automatic_mission_gazebo.sh` |
| probe | `tests/integration/test_voice_slam_session_orchestrator.py::SessionProbe` |
| 报告 | `logs/showcase/autonomous_runtime/automatic_mission_report.json` |

重型门禁只把声学 ASR 替换成 mock 文本，Gazebo、Explore Lite、SLAM Toolbox、map_saver、AMCL/Nav2、Agent NLU、ActionGuard、Action/BT/pluginlib 均真实运行。报告用 `evidence_kind` 标记证据类型，并验证 `passed/automatic_mission/map_saved`、最终 `MISSION_COMPLETED`、地图 cells、navigate/follow 成功和最终 `/cmd_vel=0`。真实麦克风准确率仍需人工演示，不能用 mock 证据替代。

### 16.3 失败定位表

| 现象 | 首查函数 | 证据 |
| --- | --- | --- |
| 无音频 | `AudioFrontendNode` 构造/PortAudio | fatal、component health |
| RMS/peak 低 | `compute_audio_frame_metrics()` | `/audio/frontend_metrics` |
| speech 无 final | `AsrEndpointRuntime.request()`、provider commit | endpoint/commit feedback |
| final 尾部丢失 | `TranscriptStabilizer.finalize()` | partial/recovered/completed |
| final 被忽略 | `ContinuousVoiceSession.accept()` | filler/duplicate/timeout |
| 多命令不拆 | `CommandNLU.parse()` | NLU event、batch metadata |
| 不入队 | `AgentControlPlane.enqueue_command()` | queue_full/retry |
| candidate 被拒 | `ActionValidator::validate()` | `/robot/action_rejected` |
| guarded command 卡住 | `ActionScheduler::enqueue()` | diagnostics pending/active |
| Action 无终态 | `SimulationControlNode::control_tick()` | feedback/BT/diagnostics |
| 小车不动 | `SimulationController::step()` | scan_timeout/front_emergency |
| 自动任务不触发 | `parse_session_command()`、`_on_asr_final()` | `/slam/session_state` |
| frontier 不结束 | `_wait_for_frontier_completion()` | status/cells/growth time |
| 存图失败 | `StageProcessManager.save_map()` | stderr、YAML/PGM |
| Nav2 不接目标 | `_wait_for_navigation_action_servers()` | Action ready/Lifecycle ACTIVE |
| 地点 unknown | `Nav2Places::to_pose_stamped()` | executor external detail |
| stop 后仍运动 | priority path、executor `stop()` | canceled result、零 `/cmd_vel` |

## 17. 函数级调用关系速查

### 17.1 普通语音动作

```text
AudioFrontendNode::input_callback
 -> AudioFrontendNode::processing_loop
 -> NlmsAudioEnhancer::process
 -> compute_audio_frame_metrics
 -> SpeechEndpointDetector::update
 -> OnlineAgentNode._on_clean_audio / OfflineAgentNode._on_audio
 -> QwenRealtimeAsr.push_audio / OfflineAgentNode._run_asr
 -> AsrEndpointRuntime.request
 -> provider.commit
 -> OnlineAgentNode._on_asr_final / OfflineAgentNode._on_asr_final
 -> TranscriptStabilizer.finalize
 -> AgentRosIo.publish_asr_final
 -> AgentApplicationRuntime.accept_transcript
 -> AgentControlPlane.accept_transcript
 -> AgentApplicationRuntime.enqueue_continuous_command
 -> AgentControlPlane.enqueue_command
 -> CommandNLU.parse
 -> ContinuousCommandQueue.put/get
 -> AgentExecutionRuntime._run_worker
 -> AgentApplicationRuntime.run_queued_turn
 -> AgentApplicationRuntime.publish_actions
 -> SequentialActionPublisher.publish
 -> action_command_to_message
 -> ActionGuardNode::on_candidate
 -> ActionValidator::validate
 -> ActionGuardNode::flush_outbox
 -> TypedActionBridgeNode::on_command
 -> ActionScheduler::enqueue
 -> TypedActionBridgeNode::dispatch_goal
 -> SimulationControlNode::handle_accepted
 -> CommandBehaviorTree::start/tick
 -> GazeboRobotExecutor::execute / Nav2RobotExecutor::execute
 -> SimulationController::step / Nav2 Action
```

### 17.2 自动建图导航

```text
AgentRosIo.publish_asr_final
 -> SessionOrchestratorNode._on_asr_final
 -> parse_session_command
 -> SessionOrchestratorNode._enqueue
 -> SessionOrchestratorNode._worker_loop
 -> SessionOrchestratorNode._execute_request
 -> SessionOrchestratorNode._run_automatic_mission
 -> parse_mapping_bootstrap_route
 -> SessionOrchestratorNode._run_agent_text_action(move/turn)
 -> StageProcessManager.start_explorer
 -> SessionOrchestratorNode._wait_for_frontier_completion
 -> exploration_completion_reason
 -> SessionOrchestratorNode._save_map
 -> StageProcessManager.save_map
 -> SessionOrchestratorNode._start_navigation
 -> SessionOrchestratorNode._wait_for_navigation_action_servers
 -> SessionOrchestratorNode._run_agent_text_action
 -> /agent/text_input
 -> 普通语音动作链的 Agent 之后部分
 -> Nav2RobotExecutor::send_navigate_goal/send_follow_goal
```

## 18. 关键技术选择与当前边界

| 选择 | 当前方案 | 替代方案 | 关键差异 |
| --- | --- | --- | --- |
| 控制协议 | typed msg/action | String+JSON | typed 可编译检查、适合 C++/DDS/rosbag |
| 固定命令 | n-gram+槽位，低置信回 LLM | 全部 LLM | 当前更低延迟、可测、可解释 |
| 长任务 | ROS 2 Action | topic/service | Action 有 feedback/cancel/result |
| 执行编排 | BT + pluginlib | 巨型 if/else 节点 | 阶段、后端、安全边界更清楚 |
| 探索结束 | status+覆盖+plateau | 固定时间 | 当前有覆盖证据且能处理不可达 frontier |
| SLAM→AMCL | 停阶段再重启 | graph 内热切换 | 当前稍慢但 TF/Lifecycle 更确定 |

当前边界：

- 演示完全在 Gazebo，不是 UART/SPI 真机验收。
- `GazeboRobotExecutor` 的 navigation 是替代运动；真实规划用 `Nav2RobotExecutor`。
- GTSAM、激光回环、动态障碍模块有独立实验，不是自动办公演示默认链路。
- NLMS AEC 已实现；WebRTC NS/AGC seam 存在但当前不 active。
- 自动门禁使用 mock 声学输入以稳定验证机器人自治；真实麦克风必须另做人工验收。
- 语义地点来自 places YAML 的 map pose，不是视觉检测实时生成。
- Explore Lite 是固定第三方依赖，首次运行先执行 setup 脚本。

## 19. 建议的代码讲解顺序

1. `voice_slam_nav_showcase.sh`：auto/mapping/save/navigation 四阶段。
2. `audio_frontend_node.cpp`：`input_callback()` 与 `processing_loop()`。
3. 在线 `_on_clean_audio()` 或离线 `_run_asr()`：provider 差异。
4. 两个 `_on_asr_final()`：指出 `/agent/asr_final` 分叉。
5. `AgentControlPlane.accept_transcript()`：唤醒、补全、急停。
6. `CommandNLU.parse()`：锚点、n-gram、槽位、多命令。
7. `enqueue_command()` 与 `_run_worker()`：batch/FIFO/TTL。
8. `RobotCommand.msg` 与 `action_command_to_message()`：typed seam。
9. `ActionValidator::validate()`：C++ 白名单和限幅。
10. `ActionScheduler::enqueue()/complete()`：FIFO、priority、stale result。
11. `SimulationControlNode::handle_accepted()`：Action server。
12. `command_tree.xml` 与 `CommandBehaviorTree::tick()`：BT。
13. `GazeboRobotExecutor` 与 `SimulationController::step()`：scan→安全→cmd_vel。
14. `SessionOrchestratorNode._run_automatic_mission()`：高层事务。
15. `_wait_for_frontier_completion()`：覆盖+plateau。
16. `_start_navigation()` 与 `_wait_for_navigation_action_servers()`：AMCL/Nav2 readiness。
17. `_run_agent_text_action()` 与 `Nav2RobotExecutor`：复用完整控制链。
18. 最后展示 `automatic_mission_report.json`，以证据结束而不是只看动画。

## 20. 验收入口

轻量状态机、阶段和取消：

```bash
source scripts/activate.sh
bash scripts/acceptance_test.sh slam-autonomous-mission-stage
```

真实 Gazebo/frontier/SLAM/map_saver/AMCL/Nav2：

```bash
source scripts/activate.sh
bash scripts/setup_frontier_exploration.sh
bash scripts/acceptance_test.sh slam-autonomous-mission
```

真实麦克风完整演示：

```bash
source scripts/activate.sh
HEADLESS=false USE_RVIZ=true \
  bash scripts/acceptance_test.sh voice-slam-workplace-demo offline
```

在线模式把最后参数改为 `online`，并保证在线密钥、网络和 provider preflight 通过。推荐只说：“小智，开始自动巡检建图”；取消说：“急停”。

最终验收不能只看 Gazebo 动画，还要同时满足：

1. `/slam/session_state` 到 `MISSION_COMPLETED`。
2. 保存目录存在 `.yaml` 与 `.pgm`。
3. `/agent/action_candidate` 含 navigate_to、follow_waypoints。
4. `/robot/action_result` 对对应 command_id 成功。
5. `/map` known/occupied cells 有效并达阈值。
6. 最终 `/cmd_vel` 的 linear/angular 均为零。
7. `automatic_mission_report.json` 的 `passed=true`。
