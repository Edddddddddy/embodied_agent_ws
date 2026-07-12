# 最终架构图与端到端数据流图

这份文档是项目汇报和面试走读的“总图入口”。README 保留快速跑通说明，
`LEARNING_NOTES.md` 解释技术细节；本文件只回答两个问题：

- 系统由哪些模块组成，在线/离线、ROS 2/C++、Gazebo/Nav2 分别在哪里？
- 一条真实语音命令从麦克风进入，到仿真机器人执行，再到日志/报告留证，经过哪些接口？

## 1. 最终系统架构图

```mermaid
flowchart TB
  User["用户 / 麦克风 / mock 文本"]

  subgraph VoiceFrontend["语音前端层"]
    MicPreflight["WSL/PulseAudio preflight"]
    AudioFrontend["C++ audio_frontend<br/>clean_pcm / endpoint / metrics"]
    VadSidecar["Silero/WebRTC VAD sidecar<br/>可选成熟端点检测"]
    KwsSidecar["KWS sidecar<br/>openWakeWord / sherpa / livekit seam"]
    Calibration["audio_frontend_calibration<br/>profile / threshold / next_command"]
  end

  subgraph AgentLayer["Agent 与语义层"]
    OnlineAgent["Online Agent<br/>DashScope/Qwen ASR-LLM-TTS"]
    OfflineAgent["Offline Agent<br/>Sherpa ASR / llama.cpp / Sherpa-TTS"]
    Session["ContinuousVoiceSession<br/>wake / sleep / duplicate / filler"]
    Queue["ContinuousCommandQueue<br/>FIFO / TTL / priority stop"]
    NLU["Command NLU + normalizer + completion<br/>多命令 / 模糊词 / 短命令补全"]
    Memory["User memory + speaker identity seam<br/>用户画像 / 行为习惯"]
  end

  subgraph RosCppLayer["ROS 2 / C++ 安全与接口层"]
    Candidate["/agent/action_candidate"]
    ActionGuard["C++ ActionGuard<br/>白名单 / 限幅 / 强类型转换"]
    RobotCommand["RobotCommand.msg"]
    ActionBridge["C++ typed_action_bridge + ActionScheduler<br/>FIFO / 抢占 / Action Client / diagnostics"]
    DemoClient["typed_action_demo_client<br/>最小 rclcpp_action client"]
  end

  subgraph SimulationLayer["仿真执行层"]
    ActionServer["simulation_control lifecycle node<br/>ExecuteRobotCommand server"]
    BT["BehaviorTree.CPP<br/>检查动作 / 检查雷达 / 执行 / 超时停止"]
    Plugins["pluginlib RobotExecutor<br/>Mock / Gazebo / Nav2"]
    Gazebo["Gazebo / TurtleBot3<br/>/cmd_vel / /odom"]
    Nav2["Nav2 NavigateToPose / FollowWaypoints<br/>语义地点 / 多目标巡航"]
  end

  subgraph EvidenceLayer["验收与证据层"]
    Monitor["continuous_voice_monitor<br/>session / queue / action / result"]
    ReleaseGate["showcase_release_gate<br/>logs/acceptance_report.json"]
    OfflineReport["offline_showcase_report<br/>模型大小 / 版本 / 解析准确率"]
    LiveReport["continuous-live-check / nav2-live-check<br/>真实麦克风现场报告"]
  end

  User --> MicPreflight
  User --> AudioFrontend
  AudioFrontend --> VadSidecar
  VadSidecar --> OnlineAgent
  VadSidecar --> OfflineAgent
  AudioFrontend --> OnlineAgent
  AudioFrontend --> OfflineAgent
  KwsSidecar --> Session
  Calibration -.-> AudioFrontend

  OnlineAgent --> Session
  OfflineAgent --> Session
  Memory -.-> OnlineAgent
  Memory -.-> OfflineAgent
  Session --> Queue
  Queue --> NLU
  NLU --> Candidate

  Candidate --> ActionGuard
  ActionGuard --> RobotCommand
  RobotCommand --> ActionBridge
  DemoClient -.-> ActionServer
  ActionBridge --> ActionServer

  ActionServer --> BT
  BT --> Plugins
  Plugins --> Gazebo
  Plugins --> Nav2
  Nav2 --> Gazebo

  Session -.-> Monitor
  Queue -.-> Monitor
  ActionServer -.-> Monitor
  Gazebo -.-> LiveReport
  Monitor -.-> LiveReport
  ReleaseGate -.-> Monitor
  OfflineReport -.-> ReleaseGate
```

读图重点：

- Agent 不直接发 `/cmd_vel`，只产生结构化动作候选。
- C++ ActionGuard 是 LLM/自然语言输出到机器人执行之间的安全边界。
- 长动作统一走 `ExecuteRobotCommand.action`；C++ `ActionScheduler` 保证单 active goal、FIFO、抢占、超时和 result 关联。
- Gazebo、Mock、Nav2 都是 executor 插件，Agent 和 ActionGuard 不需要知道底层执行后端。
- 真实麦克风稳定性由 VAD provider、profile 校准、readiness、monitor 和 live report 共同闭环。

## 2. 端到端数据流图

```mermaid
sequenceDiagram
  autonumber
  participant User as "用户"
  participant Audio as "C++ audio_frontend"
  participant VAD as "VAD/KWS sidecar"
  participant ASR as "ASR provider"
  participant Agent as "Online/Offline Agent"
  participant Queue as "Continuous queue"
  participant Guard as "C++ ActionGuard"
  participant Bridge as "typed_action_bridge / demo_client"
  participant Sim as "simulation_control Action server"
  participant Exec as "BT + pluginlib executor"
  participant Robot as "Gazebo/Nav2/TurtleBot3"
  participant Logs as "monitor / reports"

  User->>Audio: "语音：小智，向前走一秒"
  Audio-->>Logs: "/audio/frontend_metrics"
  Audio->>VAD: "/audio/clean_pcm"
  VAD-->>Agent: "/audio/speech_started / speech_ended"
  Audio->>ASR: "clean PCM stream"
  ASR-->>Agent: "/agent/asr_partial / asr_final"
  Agent->>Agent: "wake/session gate + ASR normalization"
  Agent->>Queue: "enqueue command(s)"
  Queue->>Agent: "next command when idle"
  Agent->>Agent: "NLU / LLM fallback / short completion"
  Agent-->>Logs: "/agent/session_state / command_queue / execution"
  Agent->>Guard: "/agent/action_candidate"
  Guard->>Guard: "validate / clamp / whitelist / command_id"
  Guard->>Bridge: "/robot/action_command_typed"
  Bridge->>Sim: "ExecuteRobotCommand goal"
  Sim-->>Bridge: "feedback: accepted/executing/stopping"
  Sim->>Exec: "BehaviorTree tick"
  Exec->>Robot: "/cmd_vel or Nav2 goal"
  Robot-->>Exec: "/odom / Nav2 result"
  Exec-->>Sim: "succeeded / canceled / blocked / timed_out"
  Sim-->>Bridge: "ExecuteRobotCommand result"
  Bridge-->>Agent: "/robot/action_result"
  Bridge-->>Logs: "/robot/action_feedback / result"
  Logs-->>User: "PASS/FAIL evidence JSON + terminal summary"
```

这张图适合按代码走读：

| 阶段 | 关键文件 | 关键函数/类 |
| --- | --- | --- |
| 音频与端点 | `src/embodied_agent_cpp/src/audio_frontend_node.cpp` | `AudioFrontendNode` |
| 成熟 VAD sidecar | `src/embodied_online_agent/embodied_online_agent/silero_vad_sidecar.py`、`webrtc_vad_node.py` | `StreamingVadEndpoint`、`WebRtcVadProvider` |
| 在线/离线 Agent | `online_agent_node.py`、`offline_agent_node.py` | `_on_asr_final()`、`_commit_asr_endpoint()`、`_run_turn()` |
| 连续语音队列 | `src/embodied_online_agent/embodied_online_agent/continuous_voice.py` | `ContinuousVoiceSession`、`ContinuousCommandQueue` |
| 动作解析 | `command_nlu.py`、`command_fallback.py`、`command_completion.py` | `CommandNLU.parse()`、`parse_fallback_actions()` |
| C++ 安全边界 | `src/embodied_agent_cpp/src/action_guard_node.cpp`、`action_validator.cpp` | `on_candidate()`、`ActionValidator::validate()` |
| C++ 动作调度 | `src/embodied_agent_cpp/src/action_scheduler.cpp` | `ActionScheduler::enqueue()`、`complete()` |
| ROS 2 Action client | `typed_action_bridge_node.cpp`、`typed_action_demo_client.cpp` | `apply_scheduler_events()`、`feedback_callback`、`result_callback`、`TypedActionDemoClient::run()` |
| ROS 2 Action server | `src/embodied_simulation/src/simulation_control_node.cpp` | `handle_goal()`、`update_active_action()`、`finish_active_action()` |
| BT/pluginlib 执行 | `command_behavior_tree.cpp`、`robot_executor_plugins.cpp` | `CommandBehaviorTree::tick()`、`GazeboRobotExecutor`、`Nav2RobotExecutor` |
| 验收留证 | `scripts/showcase_release_gate.py`、`continuous_live_check.py`、`generate_offline_showcase_report.py` | release gate、live report、offline report |

## 3. 汇报时的 3 句总结

1. 这个项目的主线不是“语音识别 demo”，而是把语音、Agent、强类型动作、安全校验、ROS 2 Action、Gazebo/Nav2 执行串成完整工程链路。
2. 机器人控制不信任 LLM 直接输出，所有动作都要经过 C++ ActionGuard、typed msg/action、BehaviorTree 和 executor 后端。
3. 在线/离线、mock/Gazebo/Nav2、真实麦克风/文本注入都走同一套核心接口，因此可以分层测试、分层演示、分层排障。
