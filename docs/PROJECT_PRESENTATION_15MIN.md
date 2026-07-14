# 15 分钟项目汇报与代码走读稿

本文档用于面试、答辩或项目展示。目标不是把所有实现细节讲完，而是在 15 分钟内讲清楚：

- 项目解决什么问题。
- 语音输入如何变成 ROS 2/Gazebo 中的机器人运动。
- 哪些部分体现 ROS 2 / C++ 工程能力。
- 哪些验收结果能证明链路真的打通。

## 1. 一句话项目介绍

本项目是一个面向 ROS 2 / C++ 求职展示的具身智能语音交互与仿真控制系统。它打通了：

```text
真实/模拟语音输入
→ ASR
→ 在线/离线 Agent
→ 轻量 NLU / LLM 动作解析
→ C++ ActionGuard 安全校验
→ C++ ActionScheduler 排队 / 抢占 / 结果关联
→ ROS 2 Action
→ BehaviorTree + pluginlib 仿真执行器
→ Gazebo / TurtleBot3 / Nav2
```

当前验收平台以 Gazebo/TurtleBot3 仿真为主，真实 UART/SPI 硬件接口作为 mock/预留，不把实体硬件作为本阶段交付边界。

## 2. 15 分钟讲解节奏

| 时间 | 讲什么 | 建议打开的文件/命令 |
| --- | --- | --- |
| 0:00 - 2:00 | 项目背景：为什么要做语音 Agent 到机器人控制的全链路 | `docs/FINAL_ARCHITECTURE_DIAGRAMS.md` 的最终架构图 |
| 2:00 - 4:00 | ROS 2 接口设计：为什么用 typed msg/action，而不是直接发 `/cmd_vel` | `docs/VOICE_TO_SIMULATION_CODE_WALKTHROUGH.md`、`src/embodied_agent_interfaces/msg/RobotCommand.msg`、`src/embodied_agent_interfaces/action/ExecuteRobotCommand.action` |
| 4:00 - 6:00 | C++ 安全边界：ActionGuard 如何校验、限幅、拒绝非法动作 | `src/embodied_agent_cpp/src/action_guard_node.cpp`、`src/embodied_agent_cpp/src/action_validator.cpp` |
| 6:00 - 8:00 | 连续语音：唤醒、去重、filler 过滤、队列、急停抢占 | `src/embodied_agent_core/embodied_agent_core/continuous_voice.py` |
| 8:00 - 10:30 | SLAM：受控漂移、回环约束、Ceres/GTSAM 后端 A/B 和地图指标 | `src/embodied_slam/src/gtsam_pose_graph.cpp`、`gtsam_scan_solver.cpp`、`logs/slam_backend_comparison.json` |
| 10:30 - 12:30 | 动态避障：current-only/CV/Kalman/IMM 消融、Nav2 costmap plugin 与重规划净空 | `src/embodied_navigation/src/dynamic_obstacle_tracker.cpp`、`logs/dynamic_obstacle_model_ablation.json` |
| 12:30 - 13:30 | C++ 调度与执行：FIFO、急停、Action、BehaviorTree、pluginlib | `src/embodied_agent_cpp/src/action_scheduler.cpp`、`src/embodied_simulation/src/simulation_control_node.cpp` |
| 13:30 - 15:00 | 现场演示与事实边界 | `dynamic-obstacle-navigation` 或 `continuous-offline` |

## 3. 现场只保留一条主路径和一条备用路径

### 3.1 主演示：真实麦克风 → 离线 Agent → Gazebo

```bash
bash scripts/acceptance_test.sh continuous-offline
```

按顺序说：

```text
小智
向右转，然后向前走一秒
走正方形
停下
退出控制
```

讲解时只跟踪一条数据流：ASR final → `AgentApplicationRuntime` → NLU/队列 → typed
`RobotCommand` → C++ ActionGuard/Scheduler → ROS 2 Action → BT/pluginlib → Gazebo。
通过标准是终端持续出现 session/queue/action/result，组合命令按 ID 顺序完成，`停下`
抢占，退出后 sleeping，最终 `/cmd_vel=0`。

演示前单独完成 5 分钟留证，不在 15 分钟汇报现场等待：

```bash
bash scripts/acceptance_test.sh continuous-voice-evidence offline
bash scripts/acceptance_test.sh runtime-evidence-summary
```

### 3.2 备用演示：无麦克风、无云密钥的确定性闭环

麦克风、网络或 Gazebo GUI 现场异常时，立即切换：

```bash
bash scripts/acceptance_test.sh continuous-multi-command
bash scripts/acceptance_test.sh navigation-demo
```

第一条证明一句话拆成多命令、FIFO 与 Action result 关联；第二条证明语义地点和巡航能进入
Nav2 executor seam。SLAM/动态避障不再现场启动重型流程，只展示预先生成的
`slam_backend_comparison.json`、`openloris/.../backend_comparison.json` 与
`dynamic_obstacle_navigation_report.json`，并对着 C++ 后端代码讲原理。

## 4. 从语音输入到仿真执行的代码走读地图

| 链路层 | 关键文件 | 关键函数/类 | 技术点 |
| --- | --- | --- | --- |
| 音频输入与 VAD | `src/embodied_agent_cpp/src/audio_frontend_node.cpp` | `AudioFrontendNode`、endpoint publisher | C++ 音频前端、VAD、`/audio/speech_started`、`/audio/speech_ended` |
| Agent 应用编排 | `src/embodied_agent_core/embodied_agent_core/agent_application_runtime.py` | `AgentApplicationRuntime.handle_transcript()`、`run_preparsed_turn()`、`publish_actions()` | online/offline 共享用例层；统一记忆命令、用户快照、连续队列和动作发布 |
| 在线 turn 数据面 | `src/embodied_online_agent/embodied_online_agent/online_agent_node.py`、`online_turn_runtime.py` | `_on_asr_final()`、`OnlineStreamingTurnRuntime.run()` | 节点负责 ROS/Lifecycle 接线，runtime 负责在线 LLM token、分句 TTS 和指标回调 |
| 离线 turn 数据面 | `src/embodied_offline_agent/embodied_offline_agent/offline_agent_node.py`、`offline_turn_runtime.py` | `_commit_asr_endpoint()`、`OfflineStreamingTurnRuntime.run()` | Sherpa endpoint、llama.cpp 流式 token、双缓冲 TTS 和离线延迟统计 |
| 连续会话 | `src/embodied_agent_core/embodied_agent_core/continuous_voice.py` | `ContinuousVoiceSession.accept()`、`ContinuousCommandQueue.put()`、`get()` | 唤醒、去重、filler、TTL、急停抢占 |
| typed 命令事件 | `embodied_agent_interfaces/msg/Command*Event.msg`、`ros_event_transport.py`、`ros_qos.py` | `queue_event_to_message()`、`execution_event_to_message()` | batch context、event reliable+volatile、state transient-local、编译期字段契约 |
| 多命令 NLU | `src/embodied_agent_core/embodied_agent_core/command_nlu.py` | `CommandNLU.parse()` | 字符级轻量模型、多命令识别、低置信度 fallback |
| 动作候选协议 | `src/embodied_agent_core/embodied_agent_core/protocol.py` | action payload helpers | Agent 输出结构化动作，不直接控制机器人 |
| 安全网关 | `src/embodied_agent_cpp/src/action_guard_node.cpp` | `on_candidate()` | Lifecycle node、白名单、限幅、拒绝非法动作 |
| typed 转换 | `embodied_agent_core/ros_action_transport.py` | `action_command_to_message()` | 将领域动作转成 typed candidate |
| C++ 安全校验 | `src/embodied_agent_cpp/src/action_validator.cpp` | `ActionValidator::validate()` | 白名单、字段约束、限幅和语义规范化 |
| C++ 动作调度 | `src/embodied_agent_cpp/src/action_scheduler.cpp` | `ActionScheduler::enqueue()`、`complete()` | 单 active goal、FIFO、急停抢占、失败清队列、稳定错误码 |
| ROS 2 Action bridge | `src/embodied_agent_cpp/src/typed_action_bridge_node.cpp` | `apply_scheduler_events()`、Action client callbacks | 调度决策适配为 goal/cancel/result，并发布标准 diagnostics |
| C++ Action demo | `src/embodied_agent_cpp/src/typed_action_demo_client.cpp` | `TypedActionDemoClient::run()` | 最小 `rclcpp_action` client，展示 goal/feedback/result 生命周期 |
| 仿真控制 | `src/embodied_simulation/src/simulation_control_node.cpp` | `handle_goal()`、`control_tick()`、`finish_active_action()` | ROS 2 Action server、Lifecycle、诊断、超时停止 |
| 执行后端 | `src/embodied_simulation/src/{gazebo,mock,nav2}_robot_executor.cpp` | `GazeboRobotExecutor`、`MockRobotExecutor`、`Nav2RobotExecutor` | pluginlib、Gazebo `/cmd_vel`、Nav2 action bridge |
| 行为树 | `src/embodied_simulation/src/command_behavior_tree.cpp` | `CommandBehaviorTree` | BehaviorTree.CPP 编排校验、执行、取消 |
| 漂移与闭环基准 | `src/embodied_slam/src/drift_model.cpp`、`closed_loop_driver_node.cpp` | `DriftModel::update()`、`ClosedLoopController::update()` | 固定 seed 可重复漂移、雷达安全暂停、同路线后端 A/B |
| GTSAM 后端 | `src/embodied_slam/src/gtsam_pose_graph.cpp`、`gtsam_scan_solver.cpp` | `GtsamPoseGraphOptimizer::optimize()`、`Compute()` | Prior/Between factors、Huber、协方差正定化、karto ScanSolver Adapter |
| SLAM 指标 | `tests/integration/test_slam_mapping_baseline.py` | `build_report()` | 优化前后 ATE、闭环误差、地图面积和保存产物 |
| 动态跟踪 | `src/embodied_navigation/src/dynamic_obstacle_tracker.cpp` | `DynamicObstacleTracker::update()` | 最近邻关联、速度平滑、置信度和 track TTL |
| 未来预测 | `src/embodied_navigation/src/constant_velocity_predictor.cpp` | `predict_constant_velocity()` | 无 ROS 纯函数、未来轨迹、不确定性半径增长 |
| 预测代价层 | `src/embodied_navigation/src/predicted_obstacle_layer.cpp` | `updateBounds()`、`updateCosts()` | Nav2 pluginlib Layer、未来 lethal cost、过期清除和重规划 |

## 5. 面试时可以重点强调的设计取舍

- 不让 LLM 直接发 `/cmd_vel`：LLM 只产动作候选，C++ ActionGuard 做安全边界，降低失控风险。
- 不只用 topic 表达长动作：移动、转向、导航都用 ROS 2 Action，天然支持反馈、取消和 result。
- 不在 Python 里维护可信动作的最终执行状态：C++ `ActionScheduler` 统一负责 FIFO、抢占和 result 关联，Python 只保留语音/NLU 层队列。
- 不把连续语音写成 Agent 私有逻辑：会话、队列、执行追踪抽到共享模块，online/offline Agent 复用同一套状态机。
- 不强依赖重型声学模型：默认用 energy VAD + 当前 ASR + 文本唤醒，先保证 WSL/Gazebo 演示可复现；openWakeWord/Silero 等作为后续 seam。
- 不把 SummerTTS 宣称为当前低延迟默认路径：它已完成 C++ ROS 服务化接入，适合展示端侧 TTS runtime 封装；当前 `<300ms` 低延迟 gate 仍以 Sherpa-TTS 路径为主。
- 不把“启动 slam_toolbox”说成自己做了 SLAM：项目自己实现受控漂移、GTSAM ScanSolver、
  协方差防护和指标报告；前端候选检测仍复用 slam_toolbox/karto，并明确说明边界。
- 不只在障碍出现后刹车：动态目标先由 C++ tracker 估计速度，预测层把未来 2 秒占用注入
  Nav2 全局 costmap；验收比较规划前后净空，而不是只看机器人最终没撞到。

## 6. 阶段版本发布前门禁

快速本地门禁：

```bash
pytest -q tests/repository src/embodied_offline_agent/test
bash tests/integration/test_acceptance_cli.sh
bash scripts/acceptance_test.sh continuous-mock
bash scripts/acceptance_test.sh continuous-multi-command
bash scripts/acceptance_test.sh navigation-demo
bash scripts/acceptance_test.sh offline-latency
bash scripts/acceptance_test.sh summer-tts-service
```

C++/ROS 2 门禁：

```bash
colcon test --packages-select embodied_agent_cpp embodied_simulation --event-handlers console_direct+
colcon test-result --verbose
bash scripts/acceptance_test.sh dynamic-obstacle-stage
bash scripts/acceptance_test.sh openloris-replay-stage
```

统一机器人能力门禁：

```bash
bash scripts/acceptance_test.sh robotics-gate
# logs/robotics_acceptance_report.json
```

演示前人工门禁：

```bash
bash scripts/acceptance_test.sh continuous-offline
bash scripts/acceptance_test.sh gazebo
bash scripts/acceptance_test.sh nav2-stage
bash scripts/acceptance_test.sh continuous-nav2-evidence offline
bash scripts/acceptance_test.sh dynamic-obstacle-navigation
```

运行时证据不能只看脚本是否存在。正式汇报前还要执行在线、离线各 5 分钟的真实麦克风
benchmark，并用 `runtime-evidence-summary` 核对 `proven/failed/missing`。当前实测边界与重新
留证命令见 [运行时证据状态](RUNTIME_EVIDENCE_STATUS.md)。

## 7. 当前边界与后续路线

当前阶段已经完成语音到仿真控制的主链路，适合作为 ROS 2 / C++ 求职项目展示。需要谨慎表述的边界：

- 实体 UART/SPI 硬件控制是 mock/预留，不是本阶段实体验收。
- LoRA 训练、Q8 量化可以作为规划和接口说明，不宣称完整复现实验指标。
- 已完成 OpenLORIS `office1-1` 接线基线和 `office1-7` 回访序列双后端报告；后者 449 个对齐
  位姿、99.753% 覆盖率，Ceres/GTSAM ATE RMSE 为 9.996/9.989 cm。最终轨迹恢复 2/2 个回访
  事件，但 accepted 非局部图边为 0，因此不能包装成“前端回环成功”或跨场景泛化证据。
- SummerTTS 已服务化，但当前 CPU 推理瓶颈仍明显，后续可做量化、缓存或更快声码器优化。
