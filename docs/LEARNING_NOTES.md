# 学习笔记：关键技术点与设计取舍

这份笔记面向复盘和面试讲解。每个技术点都按四个问题组织：

- 关键代码在哪里？
- 这一层怎么设计？
- 为什么这样设计？
- 和其他方案相比有什么区别？

## 1. ROS 2 通信模型：topic、msg、action 的分工

关键代码：

- `src/embodied_agent_interfaces/msg/RobotCommand.msg`
- `src/embodied_agent_interfaces/action/ExecuteRobotCommand.action`
- `src/embodied_agent_cpp/src/typed_action_bridge_node.cpp`
- `src/embodied_simulation/src/simulation_control_node.cpp`

设计方式：

- Agent 先发布 `/agent/action_candidate`，内容是 LLM 或 fallback parser 生成的结构化动作候选。
- C++ ActionGuard 将动作候选转为强类型 `RobotCommand`。
- typed action bridge 将 `RobotCommand` 发送为 `ExecuteRobotCommand` goal。
- simulation executor 返回 feedback/result，并驱动 `/cmd_vel`。

为什么这样设计：

- topic 适合广播状态和瞬时事件，例如 ASR final、动作候选、监控日志。
- ROS 2 Action 适合“移动一秒”“转九十度”这种有持续时间、可取消、需要反馈的动作。
- 自定义 msg/action 让动作接口可测试、可限幅、可扩展，比纯字符串事件载荷更工程化。

方案对比：

- 只用 `/cmd_vel`：简单，但 LLM 直接控制速度风险高，也难以表达执行结果。
- 只用 service：适合短请求，不适合持续动作和取消。
- 只用字符串事件 topic：开发快，但类型不安全，后期维护和测试成本高。

## 2. ActionGuard：LLM 输出和机器人执行之间的安全边界

关键代码：

- `src/embodied_agent_cpp/src/action_guard_node.cpp`
- `src/embodied_agent_cpp/include/embodied_agent_cpp/robot_command_adapter.hpp`
- `src/embodied_agent_cpp/src/robot_command_adapter.cpp`
- `src/embodied_agent_cpp/include/embodied_agent_cpp/action_validator.hpp`
- `src/embodied_agent_cpp/src/action_validator.cpp`

设计方式：

- 订阅 `/agent/action_candidate`。
- 解析动作候选。
- 校验动作类型、速度、时长、颜色、模式等字段。
- 通过后发布 `/robot/action_command_typed` 强类型 ROS 2 msg。
- 拒绝时发布 `/robot/action_rejected`。

为什么这样设计：

- 大模型输出不可完全信任，必须在进入机器人执行层前做白名单和限幅。
- 删除旧字符串动作命令入口，避免仿真/硬件执行层出现双入口。
- 使用 typed message，方便 C++、Action、仿真执行器稳定对接。

方案对比：

- 在 prompt 里约束模型：必要但不够，模型仍可能输出非法字段。
- 在执行器里校验：太晚，安全边界分散。
- 单独 ActionGuard：边界清晰，便于单测和面试讲解。

## 3. 连续语音会话：一次唤醒，多轮控制

关键代码：

- `src/embodied_online_agent/embodied_online_agent/continuous_voice.py`
- `src/embodied_online_agent/embodied_online_agent/wakeword.py`
- `src/embodied_online_agent/embodied_online_agent/wake_provider.py`
- `scripts/continuous_voice_monitor.py`

设计方式：

- `ContinuousVoiceSession` 负责判断一句 ASR final 是唤醒、命令、拒绝还是休眠。
- 支持唤醒词别名，例如“小志”“晓智”。
- 支持 filler 过滤，例如“嗯”“啊”。
- 支持 duplicate window，过滤短时间重复 ASR final。
- 支持 session timeout，超时后必须重新唤醒。

为什么这样设计：

- 真实麦克风会持续产生 ASR final，如果每句话都直接进 LLM，会出现误触发和卡顿。
- 会话层把“听到了什么”和“是否应该执行”分开，便于监控和调参。
- 文本唤醒先跑通，不强依赖声学 KWS 模型，部署更稳。

方案对比：

- 每条命令都要求带“小智”：安全但体验差。
- 完全不用唤醒：误触发多，不适合现场演示。
- 声学 KWS 优先：体验更好，但依赖模型和音频环境；当前项目保留 seam，默认用文本唤醒保证可部署。

## 4. 连续命令队列与急停抢占

关键代码：

- `src/embodied_online_agent/embodied_online_agent/continuous_voice.py`
- `src/embodied_online_agent/embodied_online_agent/online_agent_node.py`
- `src/embodied_offline_agent/embodied_offline_agent/offline_agent_node.py`

设计方式：

- 普通命令进入 `ContinuousCommandQueue`，按 FIFO 顺序执行。
- worker 线程逐条调用 `_run_turn()`。
- 命令执行前发布 started，执行后发布 finished。
- `停下/急停` 是 priority stop：清空队列、取消当前 sequence、立即发布 stop。
- 非优先命令设置 TTL，太旧会过期丢弃并上报。

为什么这样设计：

- 用户会连续说多条命令，不能因为上一条动作 busy 就静默丢弃下一条。
- 急停不能排队等待，必须抢占。
- TTL 防止机器人执行用户很久之前说过、已经过时的命令。

方案对比：

- busy 时直接丢弃：实现简单，但体验像“卡住”。
- 并行执行所有命令：机器人动作冲突，安全性差。
- FIFO + priority stop：兼顾连续体验和安全边界。

## 5. VAD、endpoint 与 ASR commit delay

关键代码：

- `src/embodied_agent_cpp/src/audio_frontend_node.cpp`
- `src/embodied_agent_cpp/src/audio_processing.cpp`
- `src/embodied_online_agent/embodied_online_agent/online_agent_node.py`
- `src/embodied_offline_agent/embodied_offline_agent/offline_agent_node.py`
- `scripts/audio_frontend_calibration.py`

设计方式：

- C++ audio frontend 发布 `/audio/clean_pcm`、`/audio/speech_started`、`/audio/speech_ended`、`/audio/silence_timeout`。
- Agent 收到 endpoint 后调用 ASR commit。
- `asr_commit_delay_ms` 允许在 endpoint 后等待少量时间，再提交 final。
- `VOICE_CONTROL_PROFILE` 提供 quiet、normal、noisy_room 三种参数预设。

为什么这样设计：

- 真实 ASR 容易漏掉尾部数字和量词，例如“左转90度”只 final 成“左转”。
- 适当延迟 300～500ms 可以换取更完整的识别结果。
- VAD 和 commit 分离，便于定位“音频没听到”和“ASR final 太早”两类问题。

方案对比：

- 极短静音阈值：响应快，但尾部漏识别多。
- 很长静音阈值：完整但交互迟钝。
- profile + commit delay：保留可调空间，适合不同环境。

## 6. 短命令补全与模糊归一化

关键代码：

- `src/embodied_online_agent/embodied_online_agent/command_normalizer.py`
- `src/embodied_online_agent/config/command_normalization_zh.yaml`
- `src/embodied_online_agent/embodied_online_agent/command_completion.py`
- `src/embodied_online_agent/test/test_command_completion.py`

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
- `src/embodied_online_agent/prompts/system_prompt_zh.txt`
- `src/embodied_online_agent/embodied_online_agent/protocol.py`

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

- `src/embodied_online_agent/embodied_online_agent/command_nlu.py`
- `src/embodied_online_agent/config/command_nlu_zh.json`
- `scripts/train_command_nlu.py`
- `tests/integration/test_continuous_multi_command.py`

设计方式：

- 使用字符 n-gram 原型模型识别控制意图，不依赖 torch/transformers。
- 输入一条 ASR final，输出多个动作片段和置信度。
- 例如“向右转，向前走一秒”会输出 `turn -> move`。
- 每个队列项带 `batch_id / batch_index / batch_size`，便于 monitor 解释顺序。
- 每个动作候选带 `request_id`，ActionGuard 映射成 `RobotCommand.command_id`，用于 result 关联。

为什么这样设计：

- 纯字符串 split 对无标点语音不稳，例如“向右转向前走一秒”。
- 大模型理解更强，但慢、不可预测、在线成本高。
- 轻量 NLU 覆盖固定机器人动作域，速度快、可测试、可解释。

方案对比：

- 规则拆分：部署最简单，但表达能力弱。
- 大模型 function calling：泛化强，但响应和稳定性受模型影响。
- 本地轻量 NLU + ActionGuard：在固定动作域内更适合端侧演示。

## 9. 声纹识别与用户行为记忆

关键代码：

- `src/embodied_online_agent/embodied_online_agent/speaker_identity_node.py`
- `src/embodied_online_agent/embodied_online_agent/user_memory.py`
- `src/embodied_online_agent/embodied_online_agent/online_agent_node.py`
- `src/embodied_offline_agent/embodied_offline_agent/offline_agent_node.py`
- `tests/integration/test_speaker_memory_mock.py`

设计方式：

- 声纹识别被做成 sidecar：订阅 `/audio/clean_pcm` 和 `/audio/speech_ended`，发布 `/agent/speaker_identity`。
- 声纹录入通过 `/agent/speaker_enroll_request` 触发，sidecar 把后续语音段保存成 wav 样本并维护 `speakers.txt`。
- Agent 只消费稳定 JSON identity，不直接绑定某个模型库。
- `UserMemoryStore` 按 `speaker_id` 保存本地 profile，包括用户名、偏好、常用动作、最近交互。
- Agent 推理前把当前用户画像追加进 system prompt，但动作仍必须经过 ActionGuard。
- 管理命令直接在 Agent 层处理，例如“记住我，我是小李”“我喜欢慢一点”“我是谁”“清除我的记忆”。

为什么这样设计：

- 声纹模型属于可替换能力，和 ASR/LLM/动作控制主链路解耦，降低演示风险。
- 用户画像是长期稳定信息，不适合无限追加到普通对话历史里。
- 记忆写入必须可控，不能完全交给 LLM 自行决定，否则容易把误识别或幻觉写入本地 profile。
- 低置信度声纹返回 `unknown`，避免把 A 用户偏好误写到 B 用户。

方案对比：

- 直接接 mem0/Letta/Zep：记忆能力强，但偏 Web Agent/服务端框架，当前 ROS2 端侧项目会变重。
- 使用 SpeechBrain/pyannote：模型能力成熟，但依赖 PyTorch 或 HuggingFace 模型，部署复杂。
- 当前方案：mock 可自动验收，sherpa-onnx seam 可接真实端侧声纹，和已有离线技术栈一致。

验收方式：

```bash
bash scripts/acceptance_test.sh speaker-memory-mock
bash scripts/acceptance_test.sh speaker-enroll
```

该验收证明：speaker identity 进入 Agent、用户偏好落盘、动作执行后更新用户行为统计，并且声纹录入 seam 能采集样本文件。

## 10. 离线 Agent：Sherpa、llama.cpp、Sherpa-TTS 与双缓冲

关键代码：

- `src/embodied_offline_agent/embodied_offline_agent/offline_agent_node.py`
- `src/embodied_offline_agent/embodied_offline_agent/providers/sherpa_asr.py`
- `src/embodied_offline_agent/embodied_offline_agent/providers/llama_cpp.py`
- `src/embodied_offline_agent/embodied_offline_agent/providers/sherpa_tts.py`
- `src/embodied_offline_agent/embodied_offline_agent/double_buffer.py`
- `src/embodied_offline_agent/embodied_offline_agent/latency.py`

设计方式：

- Sherpa ZipFormer 负责流式 ASR。
- llama.cpp server 提供 OpenAI-compatible completion。
- Sherpa-TTS 做本地语音合成。
- 双缓冲把 LLM 文本生成与 TTS 音频输出解耦。
- latency 模块记录离线端到端耗时。

为什么这样设计：

- 端侧算力有限，离线链路必须控制模型体积和串行等待。
- llama.cpp、Sherpa 都是轻量部署方案，适合 CPU/边缘端演示。
- 双缓冲可以减少“LLM 等 TTS / TTS 等 LLM”的卡顿。

方案对比：

- 全部云端：效果强，但不体现端侧部署能力。
- Python 大模型框架直接推理：开发方便，但部署和性能压力更大。
- llama.cpp + Sherpa：工程味更强，适合展示端侧推理思路。

## 11. BehaviorTree.CPP 与 pluginlib 仿真执行

关键代码：

- `src/embodied_simulation/config/command_tree.xml`
- `src/embodied_simulation/src/command_behavior_tree.cpp`
- `src/embodied_simulation/include/embodied_simulation/robot_executor.hpp`
- `src/embodied_simulation/src/robot_executor_plugins.cpp`
- `src/embodied_simulation/src/simulation_controller.cpp`

设计方式：

- BehaviorTree 负责动作执行流程：校验动作、检查安全、执行、确认结果。
- `RobotExecutor` 是统一接口。
- pluginlib 提供 `GazeboRobotExecutor` 和 `MockRobotExecutor` 两种后端。
- `SimulationController` 负责把动作转换成 `/cmd_vel`，并处理基础安全逻辑。

为什么这样设计：

- BT 把流程从 if/else 里抽出来，更接近 Nav2 的工程风格。
- pluginlib 让 mock 和 Gazebo 后端可替换，测试不必依赖 Gazebo。
- executor 分层后，未来接真实硬件或 Nav2 行为树更自然。

方案对比：

- 单个节点写死所有逻辑：短期快，但难测试、难扩展。
- 直接引入完整 Nav2：功能强，但本项目目标不是复杂导航，成本过高。
- 轻量 BT + pluginlib：足够展示工程规范，同时保持项目可跑通。

## 12. 测试体系

关键代码：

- `tests/repository/test_repository_layout.py`
- `tests/integration/test_acceptance_cli.sh`
- `tests/integration/test_continuous_voice_control.py`
- `src/embodied_online_agent/test/`
- `src/embodied_offline_agent/test/`
- `src/embodied_agent_cpp/test/`
- `src/embodied_simulation/test/`
- `scripts/acceptance_test.sh`

设计方式：

- repository test 保证文件结构和文档入口不漂移。
- Python 单测覆盖 Agent 侧规则、会话、队列、补全。
- C++ 单测覆盖 validator、adapter、仿真执行器。
- smoke script 覆盖 ROS 2 topic/action/launch 组合。
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

- `src/embodied_online_agent/embodied_online_agent/navigation_phrases.py`
- `src/embodied_online_agent/embodied_online_agent/command_nlu.py`
- `src/embodied_online_agent/embodied_online_agent/command_fallback.py`
- `src/embodied_agent_interfaces/msg/RobotCommand.msg`
- `src/embodied_agent_cpp/src/action_validator.cpp`
- `src/embodied_agent_cpp/src/robot_command_adapter.cpp`
- `src/embodied_simulation/include/embodied_simulation/nav2_places.hpp`
- `src/embodied_simulation/config/places.yaml`
- `src/embodied_simulation/src/simulation_control_node.cpp`
- `src/embodied_simulation/src/robot_executor_plugins.cpp`
- `src/embodied_simulation/launch/voice_nav2_turtlebot3.launch.py`
- `scripts/continuous_nav2_voice_control.sh`
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
- `tests/repository/test_repository_layout.py` 会检查 Agent 地点词表、ActionGuard 白名单和
  `places.yaml` 的 canonical place 完全一致，避免“语音能解析但 Nav2 不认地点”的漂移。
- mock/Gazebo executor 用“运动窗口”模拟导航和巡航，确保 `/cmd_vel`、Action feedback、Action result 可观测。
- `Nav2RobotExecutor` 作为 pluginlib 插件调用 Nav2 `NavigateToPose / FollowWaypoints` action；
  `nav2-bridge` 用 fake Nav2 action server 自动验证 goal 内容。
- `RobotExecutor::external_action_update()` 让 Nav2 action result 反向驱动本项目
  `ExecuteRobotCommand` 的 result，避免只按本地 `duration_s` 假完成。
- `voice_nav2_turtlebot3.launch.py` 复用官方 `nav2_bringup/tb3_simulation_launch.py`，
  再叠加本项目的 Agent、ActionGuard、typed action bridge 和 Nav2 executor。
- `nav2-turtlebot3` 重型验收会启动真实 TurtleBot3/Nav2 仿真，注入语音文本命令，
  等待目标点导航/巡航 result，并检查 `/odom` 运动证据。
- `test_continuous_navigation_queue.py` 是介于普通连续队列测试和真实 Nav2 重型测试之间的
  自动回归：它验证一次唤醒后，多目标点导航和巡航命令都能进入连续队列，并按 request_id
  对应到 ROS 2 Action result。
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

## 14. 面试讲法建议

可以用这条主线介绍项目：

> 我做的是一个 ROS 2 机器人智能语音控制系统。前端用 ASR 把语音转文本，Agent 负责唤醒、连续会话、命令归一化、LLM 动作解析和 TTS。动作不会直接控制机器人，而是先进入 C++ ActionGuard 做校验和限幅，再转换成自定义 RobotCommand 和 ROS 2 Action。仿真侧用 BehaviorTree.CPP 编排安全检查、执行和结果确认，用 pluginlib 切换 mock/Gazebo executor。最终在 Gazebo/TurtleBot3 里验证 `/cmd_vel` 和 odom 变化。

强调点：

- 不是只调 API，而是打通了 ROS 2 端到端控制链路。
- 不是 LLM 直接发速度，而是有 ActionGuard 和强类型 Action。
- 不是只写 demo，而是有连续语音、队列、急停、验收脚本和测试体系。
- 不是复杂导航项目，当前重点是语音到动作到仿真控制的闭环。
