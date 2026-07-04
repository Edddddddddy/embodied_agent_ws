# 测试与验收

本文是唯一的测试入口。原则是先验证纯逻辑，再验证 ROS 通信，最后才消耗云 API、加载
本地模型或启动 Gazebo。

## 1. 一键入口

```bash
cd /home/ubuntu/embodied_agent_ws
source scripts/activate.sh

bash scripts/acceptance_test.sh mock
bash scripts/acceptance_test.sh online
bash scripts/acceptance_test.sh offline
bash scripts/acceptance_test.sh demo
bash scripts/acceptance_test.sh continuous-mock
bash scripts/acceptance_test.sh continuous-endpoint
bash scripts/acceptance_test.sh continuous-soak
bash scripts/acceptance_test.sh continuous-queue-full
bash scripts/acceptance_test.sh continuous-ttl
bash scripts/acceptance_test.sh continuous-timeout
bash scripts/acceptance_test.sh continuous-kws-mock
bash scripts/acceptance_test.sh provider-preflight
bash scripts/acceptance_test.sh gazebo
bash scripts/acceptance_test.sh gazebo-voice
bash scripts/acceptance_test.sh all

# 交互式，不包含在 all 中
bash scripts/acceptance_test.sh microphone-offline
bash scripts/acceptance_test.sh microphone-online
bash scripts/acceptance_test.sh continuous-offline
bash scripts/acceptance_test.sh continuous-online
```

`mock` 是每次提交前的最低门槛；`demo` 使用 mock executor 验证组合动作、accessory ACK
和弧线速度；`continuous-mock` 验证一次唤醒、多命令队列、退出控制、常见 ASR 错词
归一化和 online/offline 状态机复用；`continuous-endpoint` 不走 `/agent/text_input`，
而是用 mock ASR 脚本验证 `/audio/speech_ended -> ASR commit -> /agent/asr_final`
的真实端点路径在 busy 时仍会入队；`continuous-soak` 模拟一次唤醒后的长会话，
连续输入直行、后退、转向、绕圈、挥手和灯光命令，验证 busy 时后续命令持续入队并按序
执行；`continuous-queue-full` 将队列容量设为 1，
验证说太快时第二条普通命令发布 `rejected/queue_full` 和 `queue_rejected` feedback；
同时验证队列满时“急停”仍能清空等待队列并发布 `stop`；
`continuous-ttl` 验证 Agent 忙于组合动作时，
队列里的陈旧普通命令会发布 `expired` 并跳过执行；`continuous-timeout` 验证会话窗口
超时后普通命令被拒绝，重新带唤醒词后才执行；`continuous-kws-mock` 验证 `keyword_wake` sidecar
真实打开 Agent 会话并执行动作。当前记录以 `colcon test-result --verbose` 输出为准。
`provider-preflight` 不启动 ROS，只检查可选 Silero/openWakeWord/LiveKit/sherpa provider
的 Python 依赖和关键模型路径，适合真实麦克风演示前快速失败。
`online` 使用少量
DashScope token；`offline` 会启动 llama-server；Gazebo 模式用 ACK 与里程计位移验真。
`all` 是完整自动 release gate，包含 mock、在线、离线、Gazebo 和在线/离线语音到 Gazebo，
但明确排除必须由真人说话的麦克风验收。运行 `--help` 可查看模式语义。

## 2. 分层测试矩阵

| 层级 | 主要文件/脚本 | 证明什么 |
|---|---|---|
| C++ 单元 | `embodied_agent_cpp/test/` | AEC/VAD、动作 schema、CRC、UART PTY |
| Python 单元 | `embodied_online_agent/test/` | 流式解析、记忆、唤醒、重试、fallback |
| 离线单元 | `embodied_offline_agent/test/` | 双缓冲、指标、ZipFormer 热词参数 |
| 仿真单元 | `embodied_simulation/test/` | 速度限制、雷达停车、避障、沿墙 PID |
| ROS mock | `smoke_test*.sh` | 话题发现、动作发布、ACK、watchdog |
| 音频端点 | `smoke_test_audio_endpoint.sh` | speech endpoint、VAD 参数和 AudioEnhancer 参数 seam |
| Lifecycle | `smoke_test_lifecycle.sh` | 未激活门控、激活执行、停用零速和 cleanup |
| 类型兼容 | `smoke_test_typed_action.sh` | 旧 JSON 与 typed command 同时发布且字段等价 |
| Action 状态 | `smoke_test_typed_action_server.sh` | 成功、反馈、取消、阻塞、超时和抢占 |
| BT 编排 | `test_command_behavior_tree.cpp` | 验证、反应式安全、取消、超时与恢复 |
| executor 插件 | `test_robot_executor_plugins.cpp` | 两个 pluginlib adapter 可发现且行为一致 |
| 参数与 lint | `test_node_configuration.cpp`、ament lint | 无效控制参数在 configure 前失败，产品 C++/CMake/XML 可静态检查 |
| mock 插件全链 | `smoke_test_mock_executor.sh` | 不改 Guard/BT 即可切换 backend |
| 组合演示 | `smoke_test_demo_sequence.sh` | `set_led → wave → move → turn → arc → stop` 顺序执行 |
| 连续语音 | `smoke_test_continuous_voice.sh` | 一次唤醒后多条命令排队，急停抢占，最终 `/cmd_vel` 归零 |
| 连续端点 ASR | `smoke_test_continuous_endpoint_asr.sh` | `/audio/speech_ended` 触发 mock ASR final，busy 时进入连续命令队列 |
| 连续长会话 | `smoke_test_continuous_voice_soak.sh` | 一次唤醒后持续输入多动作命令，busy 时排队并等待 Action result 后串行执行 |
| 连续队列 TTL | `smoke_test_continuous_command_ttl.sh` | 长组合动作占用 worker 时，陈旧普通命令发布 `expired` 且不执行 |
| 连续会话超时 | `smoke_test_continuous_session_timeout.sh` | `voice_session_timeout_s` 后普通命令拒绝，重新唤醒后执行 |
| provider 预检 | `voice_provider_preflight.py` | 可选 VAD/KWS provider 缺依赖、缺模型路径时提前失败 |
| 组件化等价 | `smoke_test_composed_executor.sh` | 同一控制实现可在多线程 component container 中完成 Action/BT 链 |
| 命名空间 | `smoke_test_namespaced_executor.sh` | 相对名称、Action、BT、速度和 diagnostics 均隔离到 `/robot1` |
| Action 全链 | `smoke_test_typed_action_pipeline.sh` | JSON -> typed -> Action -> 仿真控制 |
| Action + Gazebo | `smoke_test_gazebo_typed_action.sh` | terminal result 与真实 `/odom` 位移 |
| provider | `smoke_test_online_real.sh`、`benchmark_offline.sh` | 云/本地模型可用和真实延迟 |
| 物理仿真 | `smoke_test_gazebo*.sh` | Gazebo 执行动作并产生合理 `/odom` |
| 人工声学 | `accept_voice_simulation_microphone.sh` | WSLg 麦克风、真实人声和重试体验 |
| 仓库约束 | `tests/repository/` | 用户脚本与集成探针分区、关键探针保持可发现 |
| 验收 CLI | `tests/integration/test_acceptance_cli.sh` | release gate 与交互式入口保持可发现、返回码稳定 |

直接运行测试：

```bash
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose

pytest -q src/embodied_online_agent/test src/embodied_offline_agent/test
pytest -q tests/repository
bash scripts/smoke_test_recognition_retry.sh
bash scripts/smoke_test_audio_endpoint.sh
```

## 3. 真实麦克风验收

先确认设备：

```bash
pactl list short sources
pactl list short sinks
```

核心链默认关闭唤醒词与扬声器，减少两个声学变量：

```bash
bash scripts/accept_voice_simulation_microphone.sh offline
bash scripts/accept_voice_simulation_microphone.sh online
```

系统依次要求：ASR final、action candidate、guarded command、simulation ACK、非零
`/cmd_vel` 和合理 `/odom` 位移。只看到识别文本不算通过。

`voice_turtlebot3.launch.py` 默认启用 typed Action。`acceptance_test.sh gazebo` 会同时验证
默认链与显式 typed terminal result；`gazebo-voice` 要求离线语音链收到成功 result 后才通过。
typed Action 验收同时要求 `/robot/bt_status` 到达 `confirm/succeeded`，因此不是只验证 XML
能加载，而是验证真实动作确实穿过了整棵树。
所有主 launch 默认由 Nav2 lifecycle manager 自动配置并激活 Guard/仿真执行器；
`lifecycle_autostart:=false` 可用于手工检查未激活状态不会执行动作。
命名空间验收会等待 diagnostics 确认 Lifecycle 已进入 active 后再发目标，避免把“节点已
发现”误当成“控制器已就绪”的启动竞态。

单独验收唤醒词：

```bash
WAKE_WORD_ENABLED=true SPEAKER_ENABLED=false \
  bash scripts/accept_voice_simulation_microphone.sh offline
```

若门控未通过，`/agent/recognition_feedback` 会给出原因和次数，节点保持监听。热词表位于
`src/embodied_offline_agent/config/hotwords_zh.txt`。合成短词仍可能把“小智”识别成
“早日”，因此正式产品需要独立 KWS 并统计 false accept / false reject。

命令错词归一化：

```bash
pytest -q src/embodied_online_agent/test/test_command_normalizer.py
bash scripts/acceptance_test.sh continuous-mock
ros2 topic echo /agent/recognition_feedback
```

`CommandNormalizer` 在 wake/session/priority_stop 判定前运行，能把“钱进一秒”“作转九十度”
“让圈”“亭下”等常见 ASR 错词转成规范命令。`continuous-mock` 会故意发送这些错词，
并要求 online/offline 都发布 `command_normalized` 反馈和正确动作序列。
默认错词表是 `src/embodied_online_agent/config/command_normalization_zh.yaml`，安装后会随
`embodied_online_agent` 包一起进入 share 目录。真实麦克风测试中发现新错词时，优先复制
该 YAML 并用 `command_normalization_path:=/path/to/your.yaml` 覆盖；如果离线 ASR 总是把
某个控制短语听歪，再同步补充 `src/embodied_offline_agent/config/hotwords_zh.txt`。
连续语音脚本等价支持 `COMMAND_NORMALIZATION_PATH`、
`COMMAND_NORMALIZATION_FUZZY_THRESHOLD`、`COMMAND_NORMALIZATION_ENABLED` 和
`COMMAND_NORMALIZATION_FEEDBACK_ENABLED`，用于真实麦克风现场快速试错。
默认不强制安装外部依赖；如需启用成熟 RapidFuzz scorer，可执行：

```bash
source .venv/bin/activate
pip install -e "src/embodied_online_agent[fuzzy]"
```

真实麦克风环境建议先做 profile 校准：

```bash
# Terminal 1：启动连续语音控制
bash scripts/continuous_voice_control.sh offline

# Terminal 2：采样 /audio/frontend_metrics，并按 quick apply 设置 profile
python3 scripts/audio_frontend_calibration.py --duration 6
export VOICE_CONTROL_PROFILE=noisy_room
```

校准输出中的 `recommended VOICE_CONTROL_PROFILE` 是连续语音脚本的现场预设建议：
`quiet` 适合输入偏弱或 VAD 太保守，`noisy_room` 适合持续噪声/误触发，`normal` 表示
当前音频端点较均衡。

连续语音控制：

```bash
bash scripts/continuous_voice_control.sh offline
bash scripts/continuous_voice_control.sh online

# 无需麦克风，打印实际 launch 参数，适合演示前检查环境变量覆盖是否正确
CONTINUOUS_PRINT_CONFIG=true \
  VOICE_CONTROL_PROFILE=noisy_room \
  WAKE_WORD_ENABLED=false \
  AUDIO_ENHANCER=webrtc \
  NOISE_SUPPRESSION_ENABLED=true \
  AUTO_GAIN_ENABLED=true \
  bash scripts/continuous_voice_control.sh online
```

默认参数为 `wake_word_enabled:=true`、`continuous_control_enabled:=true`、
`voice_session_timeout_s:=60.0`、`continuous_command_queue_size:=8`、
`continuous_command_max_age_s:=30.0`、
`speaker_enabled:=false`、`audio_enhancer:=nlms`、`aec_enabled:=true`、
`noise_suppression_enabled:=false`、`auto_gain_enabled:=false`、
`command_normalization_enabled:=true`、
`command_normalization_feedback_enabled:=true`、
`command_normalization_fuzzy_threshold:=0.82`。这些值都可通过
同名大写环境变量覆盖。`VOICE_CONTROL_PROFILE=normal|quiet|noisy_room` 可批量调整
VAD 端点、队列容量、旧命令 TTL 和纠错阈值；其中 `quiet` 适合安静近讲，
`noisy_room` 适合嘈杂环境减少误触发。显式环境变量优先级高于 profile 默认值。
人工连续脚本默认在 launch 后运行 `voice_control_readiness_check.py` 采样 3 秒：
通过后会提示 `系统已就绪，可以开始说：小智`；若有 blockers/warnings 会打印诊断但继续
运行，避免现场被一次短采样完全阻断。输出中的 `recommended_voice_profile` 与
`quick_apply` 可直接用于下一轮 `VOICE_CONTROL_PROFILE` 调整。可通过
`CONTINUOUS_READINESS_ENABLED=false`
关闭，或用 `CONTINUOUS_READINESS_DURATION=5` 调整采样时长。
说一次“小智”后，60 秒内可以
连续说“向前走一秒 / 左转九十度 / 绕圈 / 走正方形”等命令；Agent 忙于执行上一条时不会
丢弃新的 ASR final，而是排入队列。会话层会忽略“嗯/啊/哦/呃”等短 filler，并对短时间
重复出现的同一句 ASR final 去重；这两个规则只处理明显噪声，避免把正常的二次命令误删。
去重窗口默认 1.2 秒，可通过 `CONTINUOUS_DUPLICATE_WINDOW_S` 在连续控制脚本中调整；
休眠或重新唤醒会清空去重记忆，新会话里可以立即再次执行同一句命令。
过滤结果会发布到 `/agent/recognition_feedback`，monitor 显示为 `[ignore] filler ...`
或 `[ignore] duplicate_command ...`，因此真实麦克风验收时可以区分“系统卡住”和“系统正在
主动过滤噪声”。
普通命令在队列里等待超过 30 秒会自动过期跳过，
避免执行已经失去上下文的旧命令，并在 `/agent/command_queue` 发布 `expired` 事件；说
“停下/急停”会清空等待队列并立即发布 stop；说
“退出控制/休眠/结束控制”会关闭会话并发布安全 stop，后续命令必须重新唤醒。
`CONTINUOUS_COMMAND_QUEUE_SIZE` 可调队列容量；队列满时会发布 `rejected/queue_full`，
monitor 会显示 `[queue] rejected ...` 和 `[queue-feedback] queue_full ...`，
summary 可用于判断是否应该减慢说话节奏或调大容量。
如果 `voice_session_timeout_s` 到期，下一条不带唤醒词的普通命令也会被拒绝并进入
`retry_listening`，monitor 会显示 `[session-timeout] 会话已超时，请先说小智 ...`；
`continuous-timeout` 用 0.8 秒窗口覆盖这个行为。
验收探针还会检查 `/agent/wake_event` 中出现 `wake/continue/sleep/rejected`，以及
`/agent/session_state` 中出现 `awake/sleeping`；同时检查 `/agent/command_queue` 里有
真实 `size`，`/agent/command_execution` 里有 `started/finished`。随后会重新唤醒，发送
“走正方形”后立刻发送“急停”，要求出现 `stop` candidate、priority stop 清队列事件，
并确认最终 `/cmd_vel` 为零。
人工脚本默认启动 `scripts/continuous_voice_monitor.py`，终端会持续打印：

```text
[session] awake
[wake] text:wake
[asr] 向前走一秒
[ignore] filler 嗯。
[queue] enqueue 向前走一秒 size=1
[queue-feedback] queue_full 绕圈 size=8
[queue] expired 向前走一秒 reason=stale_command size=1
[exec] started 向前走一秒
[action] executing move
[feedback] executing 45% mock_execution
[result] succeeded
[summary] wake=1 sleep=1 retry=0 timeout=0 asr=3 ignored=1 normalized=1 enqueued=2 rejected=0 expired=0 started=2 finished=2 succeeded=2 failed=0
[summary-audio] samples=12 profile=noisy_room reason=persistent_speech_or_noise mean_rms=0.0200 max_rms=0.0210 speech_ratio=1.00 dropped_input_delta=0 warnings=vad_threshold_may_be_too_low_or_environment_noisy
```

`[summary]` 在 Ctrl-C 退出 monitor 时打印；`continuous_voice_control.sh` 的清理逻辑也会
优先用 SIGINT 结束 monitor，确保这行复盘信息尽量落盘。`wake/sleep/retry/timeout` 反映会话门控和
重试体验，`asr` 是收到的 final 数，`ignored` 是 filler/duplicate 过滤数，
`normalized` 是错词归一化数，`enqueued/rejected/expired` 反映队列健康，
`started/finished/succeeded/failed` 反映动作执行闭环。若 monitor 收到过
`/audio/frontend_metrics`，还会打印 `[summary-audio]`，其中 `profile/reason` 直接给出
下一轮真实麦克风演示应尝试的 `VOICE_CONTROL_PROFILE`。音频样本采用最近 600 条
metrics 的滑动窗口，避免长时间开麦时 monitor 内存无界增长。如果只想看 launch 原始日志，
可设置 `CONTINUOUS_MONITOR_ENABLED=false`；如果需要更长或更短的音频复盘窗口，可设置
`CONTINUOUS_MONITOR_AUDIO_SAMPLE_LIMIT=1200` 等值。

真实 KWS provider 可直接从连续控制脚本配置，不必手改 YAML。脚本会把这些值同时传给
`voice_provider_preflight.py` 与 `voice_turtlebot3.launch.py`：

```bash
# sherpa-onnx KeywordSpotter
KWS_PROVIDER=sherpa \
SHERPA_KWS_TOKENS=/models/kws/tokens.txt \
SHERPA_KWS_ENCODER=/models/kws/encoder.onnx \
SHERPA_KWS_DECODER=/models/kws/decoder.onnx \
SHERPA_KWS_JOINER=/models/kws/joiner.onnx \
SHERPA_KWS_KEYWORDS_FILE=/models/kws/keywords.txt \
  bash scripts/continuous_voice_control.sh offline

# openWakeWord；多个模型用逗号分隔
KWS_PROVIDER=openwakeword \
OPENWAKEWORD_MODELS=/models/kws/xiaozhi.onnx,/models/kws/nihaoxiaozhi.onnx \
OPENWAKEWORD_THRESHOLD=0.42 \
  bash scripts/continuous_voice_control.sh offline

# LiveKit WakeWord
KWS_PROVIDER=livekit \
LIVEKIT_WAKEWORD_MODELS=/models/kws/livekit-xiaozhi.onnx \
LIVEKIT_WAKEWORD_THRESHOLD=0.63 \
  bash scripts/continuous_voice_control.sh offline
```

`CONTINUOUS_PRINT_CONFIG=true` 会打印最终 launch 命令，并明确列出
`VOICE_SESSION_TIMEOUT`、`CONTINUOUS_COMMAND_QUEUE_SIZE`、`CONTINUOUS_COMMAND_MAX_AGE`
和 `CONTINUOUS_DUPLICATE_WINDOW_S` 等现场调参值；`CONTINUOUS_PREFLIGHT_ENABLED=true`
会在启动 ROS/Gazebo 前检查依赖和模型路径。`openwakeword_models` 与
`livekit_wakeword_models` 在 launch 中以字符串传递，`keyword_wake` 节点会按逗号拆成
模型列表，和 YAML list 写法兼容。
Silero VAD 也支持同样的“脚本环境变量覆盖 YAML”方式：

```bash
VAD_PROVIDER=silero \
SILERO_VAD_MODEL_PATH=/models/vad/silero_vad.onnx \
SILERO_VAD_USE_ONNX=true \
SILERO_VAD_THRESHOLD=0.61 \
  bash scripts/continuous_voice_control.sh offline
```

`voice_provider_preflight.py` 会优先检查这些覆盖值；launch 会把它们传给
`silero_vad` sidecar 的 `model_path/use_onnx/threshold` 参数。

VAD endpoint 与 AudioEnhancer seam：

```bash
bash scripts/smoke_test_audio_endpoint.sh
ros2 topic echo /audio/speech_started
ros2 topic echo /audio/speech_ended
ros2 topic echo /audio/frontend_metrics
```

当前 `vad_provider:=energy`，端点参数包括 `speech_end_silence_s`、`min_utterance_ms`、
`max_utterance_s`。连续控制脚本额外暴露 `SPEECH_START_THRESHOLD`，并在 launch 层映射为
C++ AudioFrontend 的 `vad_rms_threshold`；`SPEECH_END_SILENCE_S`、`MIN_UTTERANCE_MS`、
`MAX_UTTERANCE_S` 则直接透传同名 launch 参数。AudioFrontend 会同步发布旧
`/audio/silence_timeout` 以兼容已有测试；
Agent 对 `speech_ended` 与 `silence_timeout` 的同次事件做 50 ms 去重，避免双 commit。
`/audio/frontend_metrics` 每 `metrics_period_s` 秒发布一次诊断 JSON；真实麦克风排障时，
重点看 `rms` 是否明显大于静音、`speech` 是否随说话切换、`dropped_input_frames` 是否增长。
该 JSON 还会包含 `audio_enhancer_requested/audio_enhancer_active`、`aec_active`、
`noise_suppression_active`、`auto_gain_active`，用于确认当前是 NLMS AEC 还是未来
WebRTC enhancer，以及 NS/AGC 是否真正生效。
建议在长时间语音控制前先跑一次校准：

```bash
# 终端 1：启动包含 audio_frontend 的真实麦克风链路
bash scripts/continuous_voice_control.sh offline

# 如 VAD 过早/过晚断句，可先用 dry-run 确认参数，再正式启动
CONTINUOUS_PRINT_CONFIG=true \
SPEECH_START_THRESHOLD=0.021 \
SPEECH_END_SILENCE_S=0.38 \
MIN_UTTERANCE_MS=240 \
MAX_UTTERANCE_S=7.5 \
  bash scripts/continuous_voice_control.sh offline

# 终端 2：收集 8 秒指标并输出调参建议
python3 scripts/voice_provider_preflight.py --mode offline --vad-provider "${VAD_PROVIDER:-energy}" --kws-provider "${KWS_PROVIDER:-none}"
python3 scripts/audio_frontend_calibration.py --duration 8
python3 scripts/audio_frontend_calibration.py --duration 8 --json

# 同时检查音频前端与声学 KWS 分数；使用 openwakeword/livekit 时建议加 --require-kws
python3 scripts/voice_control_readiness_check.py --duration 8 --require-kws
```

常见结果解释：

- `microphone_too_quiet_or_disconnected`：麦克风输入太小或设备没接通，先检查 WSL/系统输入源。
- `vad_threshold_may_be_too_high`：有音量但 `speech=false`，降低 energy VAD 阈值。
- `vad_threshold_may_be_too_low_or_environment_noisy`：长时间 `speech=true`，提高阈值或降低环境噪声。
- `audio_input_overrun`：输入丢帧，检查 CPU 占用、音频块大小和队列。
- `tts_playback_overrun`：回放丢块，连续控制演示优先保持 `speaker_enabled:=false`。
- `audio_enhancer_fallback`：请求的增强器不可用，当前已回退到 `audio_enhancer_active`。
- `noise_suppression_unavailable` / `auto_gain_unavailable`：已请求 NS/AGC，但当前
  NLMS enhancer 不提供该能力；真实启用需要后续接 WebRTC enhancer。

Silero VAD 已作为可选 sidecar 接入：AudioFrontend 继续发布 `/audio/clean_pcm`，当
`vad_provider:=silero` 时内置 energy endpoint 自动关闭，由 `silero_vad` 节点发布
`/audio/speech_started`、`/audio/speech_ended` 与可观测 `/audio/vad_event`。

无模型依赖的配置冒烟：

```bash
bash scripts/smoke_test_silero_vad_sidecar.sh
```

真实启用前需安装可选依赖：

```bash
source .venv/bin/activate
pip install silero-vad onnxruntime
VAD_PROVIDER=silero bash scripts/continuous_voice_control.sh offline
```

当前 `audio_enhancer:=nlms`，`aec_enabled:=true` 默认启用现有 NLMS AEC；
`noise_suppression_enabled` 与 `auto_gain_enabled` 只是 WebRTC adapter 的预留参数，
现在开启会回退并告警。人工连续控制脚本会把 `AUDIO_ENHANCER`、`AEC_ENABLED`、
`NOISE_SUPPRESSION_ENABLED`、`AUTO_GAIN_ENABLED` 透传到 `voice_turtlebot3.launch.py`
和底层 `audio_frontend`，可用 `CONTINUOUS_PRINT_CONFIG=true` 先检查实际参数。

WakeProvider seam：

```bash
ros2 topic echo /agent/wake_event
ros2 topic echo /agent/session_state
ros2 topic pub --once /agent/wake_event_input std_msgs/msg/String \
  "{data: '{\"kind\":\"wake\",\"provider\":\"manual_kws\"}'}"
bash scripts/acceptance_test.sh kws-sidecar
bash scripts/acceptance_test.sh continuous-kws-mock
```

当前默认 provider 为 `text`，事件包括 `wake`、`continue`、`sleep`、`rejected`。
`/agent/wake_event_input` 是外部声学 KWS 的稳定入口；手工发布 `manual_kws` wake 后，
下一句不带“小智”的 `/agent/text_input` 也应进入同一套连续命令队列。后续
sherpa-onnx/openWakeWord adapter 只需要按这个 topic 契约发布 wake/sleep。
`continuous-mock` 还会直接向 `/agent/wake_event_input` 注入外部 sleep，
验证会话关闭和安全 `stop` 候选动作。
`keyword_wake` sidecar 当前提供三种主要模式：

- `kws_provider:=mock_text`：订阅 `/agent/kws_text_input`，无模型验收 KWS 链路。
- `continuous-kws-mock`：启动 mock KWS、online/offline mock Agent、ActionGuard 和 mock
  executor，验证“不唤醒拒绝命令 -> KWS 唤醒 -> 不带小智的命令执行”。
- `kws_provider:=sherpa`：订阅 `/audio/clean_pcm`，使用 sherpa-onnx `KeywordSpotter`；
  需要在 `keyword_wake` 参数组里配置 `sherpa_tokens/encoder/decoder/joiner/keywords_file`。
- `kws_provider:=openwakeword`：订阅 `/audio/clean_pcm`，使用 openWakeWord
  `Model.predict()` 的分数输出；需要安装可选依赖并配置 `openwakeword_models`、
  `openwakeword_threshold` 与 `openwakeword_inference_framework`。openWakeWord 官方
  预训练模型主要面向英文，中文“小智”建议使用 sherpa KWS 或后续 LiveKit/openWakeWord
  自训练模型。
- `kws_provider:=livekit`：订阅 `/audio/clean_pcm`，使用 LiveKit WakeWord
  `WakeWordModel.predict()`；需要安装可选依赖并配置 `livekit_wakeword_models` 和
  `livekit_wakeword_threshold`。该方案适合后续用 LiveKit 训练并导出中文“小智”ONNX。

openWakeWord 可选依赖安装：

```bash
source .venv/bin/activate
pip install -e "src/embodied_online_agent[kws]"
KWS_PROVIDER=openwakeword bash scripts/continuous_voice_control.sh offline
```

无真实模型依赖的 openWakeWord adapter runtime 冒烟：

```bash
bash scripts/acceptance_test.sh openwakeword-sidecar
```

该脚本会临时注入 fake `openwakeword.model.Model`，验证 ROS 节点参数、`/audio/clean_pcm`
订阅、`Model.predict()` 调用、`/agent/kws_score` 分数诊断和 `/agent/wake_event_input`
发布，不代表真实唤醒词模型效果。

LiveKit WakeWord 可选依赖与无模型 smoke：

```bash
source .venv/bin/activate
pip install -e "src/embodied_online_agent[livekit-kws]"
KWS_PROVIDER=livekit bash scripts/continuous_voice_control.sh offline

# 无真实模型依赖，只验证 ROS adapter runtime
bash scripts/acceptance_test.sh livekit-sidecar
```

openWakeWord/LiveKit 模式会发布 `/agent/kws_score`，用于调唤醒阈值：

```bash
ros2 topic echo /agent/kws_score
python3 scripts/kws_score_calibration.py --duration 8
python3 scripts/kws_score_calibration.py --duration 8 --json
```

字段包括 `top_keyword`、`top_score`、`threshold`、`above_threshold` 和完整 `scores`。
连续演示 monitor 会显示为：

```text
[kws-score] livekit_test fake_livekit_wake=0.930 threshold=0.500 above=True
```

无模型依赖的阈值校准 smoke：

```bash
bash scripts/acceptance_test.sh kws-calibration
bash scripts/acceptance_test.sh voice-readiness
```

## 4. 当前实测基线

环境：WSL Ubuntu 24.04、ROS 2 Jazzy、16 vCPU、约 8 GB RAM；最新复验日期
2026-07-02。以下为本次单次/少量样本，不是 SLA。

| 指标 | 当前样本 | 结论 |
|---|---:|---|
| 在线 LLM 冷启动首 token | 1.474 s | 不达 1 s，必须预热 |
| 在线 LLM 热启动首 token | 350–384 ms | 当前样本达标 |
| 在线 TTS 首音频 | 222–242 ms | 当前样本达标，仍需 P95 |
| 在线 ASR commit-to-final | 174 ms | 当前样本达标 |
| ZipFormer RTF | 0.0416 | 远快于实时 |
| llama.cpp CPU decode | 34.10 token/s | 超过 8.6 目标 |
| Melo/Sherpa-TTS RTF | 0.4006 | 快于实时 |
| 离线首音频 | 2.274 s | 当前样本达标 |
| 离线整轮 | 2.313 s | 当前样本低于 3.5 s |
| 消息/音频丢弃 | 0 / 0 | 当前样本无背压丢失 |
| 原始模型 / fallback 指令通过 | 2/8 / 7/8 | 链路可用；模型本身仍需 LoRA |
| Gazebo 兼容链 / typed 链位移 | 0.088 / 0.330 m | 两条链均产生真实里程计位移 |
| 离线语音→typed Action→Gazebo | 0.162 m | 带 ASR 噪声仍完成动作并返回 success |

这些是少量样本，不应包装成稳定 SLA。正式数据需要固定硬件、冻结输入集、保存原始日志，
分别报告冷/热启动 100 轮 P50/P95。

### 2026-07-02 release gate 结果

| Gate | 结果 | 关键证据 |
|---|---|---|
| `mock` | PASS | 130 colcon + 2 repository tests，全链 smoke 通过 |
| `online` | PASS | 实际 DashScope ASR、LLM、TTS 与 Guard/hardware mock |
| `offline` | PASS | ZipFormer、Q8 llama.cpp、Sherpa-TTS、双缓冲与 hardware mock |
| `gazebo` | PASS | `/scan`、`/cmd_vel`、`/odom`、Action result 与 BT confirm |
| `gazebo-voice` | PASS | speech→ZipFormer→llama.cpp/fallback→Guard→Action/BT→Gazebo |
| `gazebo-voice-online` | PASS | speech→在线 ASR/LLM→Guard→typed Action/BT→Gazebo，位移 0.163 m |
| 真人麦克风 | 待人工复验 | WSLg 已发现 `RDPSource`，但自动任务不能代替真人发声 |

## 5. 完成度与未验收项

已实测：在线/离线模型 adapter、0.4 秒断句、流式解析、双缓冲、动作 Guard、UART PTY、
Gazebo 位移、语音到仿真闭环、失败重试。

仍待实测：

- LoRA 训练及独立测试集上的模型指令遵循率；当前 8 条种子集纯模型仅 2/8。
- 真实扬声器/麦克风下的 AEC ERLE、双讲、距离和噪声测试。
- 目标机器上的 100 轮端到端 P95。
- 实体 `/dev/ttyUSB*`、`/dev/spidev*`、MCU、电机和急停。
- 独立 KWS 的误唤醒率、漏唤醒率和不同说话人测试。

## 6. 常见故障

### `ros2 topic echo` 没有输出

它是持续订阅命令，消息到来前等待是正常的。另开终端执行 `ros2 topic pub --once`，并
确认两个终端加载了同一 `scripts/activate.sh` 与 `ROS_DOMAIN_ID`。

### 正确 ASR 后没有动作

检查 `wake_word_enabled` 是否按预期传入节点，然后依次观察：

```bash
ros2 topic echo /agent/state
ros2 topic echo /agent/recognition_feedback
ros2 topic echo /agent/action_candidate
ros2 topic echo /robot/action_rejected
ros2 topic echo /robot/action_ack
```

### 一轮结束后持续识别

旧的一句话验收脚本会在六阶段成功后退出；连续控制脚本会一直监听，这是预期行为。若
你只想验证一条命令，请使用 `accept_voice_simulation_microphone.sh`。若想长时间控制，
请使用 `continuous_voice_control.sh`，并用“退出控制”关闭当前会话。

### 单测通过但真实链失败

按声卡 -> ASR -> LLM -> parser -> Guard -> executor -> odom 顺序定位，不要直接把问题
归因于模型。验收脚本的六阶段输出就是为这个目的设计的。
