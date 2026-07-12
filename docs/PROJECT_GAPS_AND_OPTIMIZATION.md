# 项目不足与优化路线

本文档记录当前项目相对“产品级语音机器人系统”的不足。它不是否定当前版本；
当前版本已经适合作为 ROS 2 / C++ 求职展示项目。本文的作用是帮助后续开发保持方向：
优先补稳定性、证据和演示可信度，而不是盲目堆功能。

## 1. 真实语音稳定性仍依赖环境

现状：

- `continuous-offline/online` 已能跑通，但 WSL 麦克风、PulseAudio/WSLg、VAD 阈值仍受环境影响。
- 连续语音脚本已默认 `VAD_PROVIDER=auto`：Silero VAD 依赖可用时优先使用成熟声学 VAD，
  不可用时尝试轻量 WebRTC VAD，最后清晰降级到 energy VAD。
- provider preflight 已能在降级到 energy 或显式 provider 缺依赖时输出推荐安装命令，
  把“发现缺依赖”推进到“知道下一步运行哪个 setup 脚本”。
- WebRTC VAD 已作为可选 sidecar 接入；Sherpa KWS 已具备可复制的 runtime 启动验收。
- KWS score calibration 已能把 openWakeWord/LiveKit 现场分数转换成下一轮阈值环境变量。
  WebRTC AEC/NS、openWakeWord 和 KWS 现场召回率仍是后续增强，不是默认强依赖。

优化：

- 继续把 Silero/WebRTC VAD 依赖安装、模型缓存和真实麦克风证据做成更稳定的默认演示路径。
- 引入声学 KWS 默认方案，优先选择部署成本低、可离线运行的方案。
- 继续把 `wsl-microphone-preflight`、`audio_frontend_calibration.py --json`、
  `recommended_environment/next_command` 和真实连续语音报告串成更闭环的一键诊断。
- 保存真实演示报告，避免“现场听起来能跑”但缺少可复查证据。

## 2. 离线端侧模型效果证据还不够硬

现状：

- llama.cpp、Sherpa-ONNX、Sherpa-TTS、SummerTTS 的工程接口已经具备。
- 模型资产、运行时版本、deterministic parser、llama.cpp decode tokens/s 和离线 LLM
  instruction-following 已有可运行报告入口；但 LoRA 微调、训练后准确率和更大规模真实 ASR
  错误集评估仍不够硬。

优化：

- 使用 `training/robot_instruction_eval.jsonl` 固化动作解析评估样例；当前已有 43 条代表集，
  覆盖 ASR 错词、多命令、Nav2 目标点/巡航、安全拒绝，以及速度/距离/角度/时长/地点槽位。
- 用 `docs/OFFLINE_BENCHMARK_REPORT.md` 记录模型大小、首 token、tokens/s、ASR/TTS
  realtime factor、`model_score/effective_score` 和未复现边界。
- 后续再补 LLaMA-Factory LoRA 训练复现实验，不把未复现指标写成已完成能力。

## 3. SummerTTS 服务化完成，但不应作为低延迟默认方案

现状：

- SummerTTS 已有命令行 provider 和常驻 C++ ROS service。
- service 能避免每句重新启动进程/加载模型，并已支持短文本缓存；未缓存文本的 CPU 合成仍是秒级。
- 当前 `<300ms` TTS gate 仍以 Sherpa-TTS 为准。

优化：

- 继续评估量化、分段合成或更快声码器。
- 文档中继续明确：SummerTTS 是 C++ runtime 集成亮点，不是当前低延迟默认路径。

## 4. Nav2 演示还不是完整导航项目

现状：

- 已支持语义地点、`navigate_to`、`follow_waypoints`、Nav2 bridge 和 TurtleBot3/Nav2 bringup。
- 已补充项目本地 `voice_demo.yaml`、`voice_demo.sdf.xacro` 和 `voice_nav2_demo.rviz`
  演示资产；目标点和场景仍比较静态。
- 已开始透传 Nav2 action 级别的失败细节，例如 server unavailable、goal rejected、aborted/canceled、
  error code/message、missed waypoints；但还没有进一步语义化归因到 planner/controller/localization
  等具体 Nav2 子系统。

优化：

- 已用 `nav2-resilience` 验证运行时动态插入障碍后的全局重规划，以及地图外目标的
  `aborted/error_code` 反馈；后续可继续丰富房间语义和更多业务目标点。
- 继续强化 RViz 展示脚本，突出 TF、map、path、goal 和导航状态。
- 继续把 Nav2 result 与 `/diagnostics`、planner/controller 日志关联，形成更细粒度报告。

## 5. NLU 多命令泛化有限

现状：

- 轻量 NLU 覆盖固定动作域，比简单字符串拆分更稳。
- 真实 ASR 错误样本还少，复杂自由表达泛化有限。

优化：

- 收集真实 ASR final，持续补进 `training/robot_instruction_eval.jsonl`，并观察 `failed_cases`
  与 tag 维度准确率。
- 对规则、轻量模型、在线 function calling 做准确率/延迟对比。
- 离线默认保留轻量方案，在线可探索更强 parser。

## 6. 测试体系强，但真实场景仍需人工证据

现状：

- 自动测试覆盖 repository、Agent、C++、ROS smoke、Nav2 mock。
- 真实麦克风、Gazebo GUI、Nav2 重型链路仍不能完全放进 CI。

优化：

- 使用 `bash scripts/acceptance_test.sh release-gate` 运行默认 5 条核心门禁并输出统一报告；
  需要更完整的本地门禁时再运行 `python3 scripts/showcase_release_gate.py --profile full`。
- 演示前再跑真实麦克风和 Nav2 evidence。
- 后续可增加录屏、截图、RViz 状态导出。

## 7. 控制面和 ROS 2 中间件仍需继续收敛

现状：

- C++ 已覆盖 ActionGuard、`ActionScheduler`、Action Client、audio frontend、simulation executor
  和 SummerTTS service；可信动作的 FIFO、抢占和 result 关联不再由 Python 最终裁决。
- `/agent/command_queue`、`/agent/command_execution` 已从 `String + JSON` 升级为自定义 msg，
  并统一 reliable QoS；Agent/session 当前状态开始使用 transient-local。
- 唤醒事件、识别反馈、音频前端指标、仿真状态等仍有结构化 JSON topic，尚未完全 typed。

优化：

- 下一步按控制风险排序，把 wake/recognition/simulation state 等跨节点结构化事件升级为 typed msg
  或标准 `/diagnostics`；普通 ASR/回复文本仍可保留 `std_msgs/String`。
- 将 `typed_action_bridge` 升级为 Lifecycle/Component node，并统一参数校验和 callback group。
- 为命令、状态、传感器流分别定义可靠性、durability、deadline/liveliness 策略，而不是统一 depth=10。
- 保留 Python 在模型编排层的灵活性，不为“全 C++”牺牲迭代速度。

## 8. 用户记忆和声纹仍需继续产品化

现状：

- 已有 speaker identity topic、user memory store、mock/sherpa seam 和记忆命令。
- “慢一点/快一点/默认前进时长/默认转角”等偏好已经能在动作出口确定性影响
  `move/turn/arc` 参数，并且仍经过 ActionGuard。
- 低置信度声纹会被归为 `unknown`，写个人画像时触发 `LowConfidenceSpeakerError`；
  Agent 捕获后跳过记忆写入，避免误识别污染用户 profile。
- 真实声纹默认链路、偏好冲突合并和可视化管理仍较浅。

优化：

- 明确用户画像 schema、导出/清理命令和更细粒度隐私策略。
- 增加用户记忆查看、导出、过期和冲突合并机制。

## 9. 文档丰富但入口偏多

现状：

- README、学习笔记、验收文档、汇报文档、路线图都比较完整。
- 新读者可能不知道先看哪份，面试时也容易讲散。

优化：

- README 保持主入口和快速跑通。
- `PROJECT_PRESENTATION_15MIN.md` 用于汇报。
- `INTERVIEW_QA.md` 用于追问。
- 深入实现继续放 `LEARNING_NOTES.md`。

## 10. 代码和脚本组织规模偏大

现状：

- online/offline Agent 主节点分别超过 1100/1200 行，provider 创建、ROS publisher、连续会话、
  memory 和 turn pipeline 混在一个类中。
- `embodied_agent_cpp` 同时承载 audio、control、hardware、TTS 等多个变化方向。
- `scripts/` 超过 120 个文件，`acceptance_test.sh` 同时承担目录、路由、环境配置和执行逻辑。

优化：

- 先抽取共享 `AgentControlPlane`，隐藏 session/queue/typed event publisher，online/offline 只保留
  provider 与 turn pipeline 差异；避免继续复制同名私有函数。
- 等 control/audio/hardware 各自接口稳定后，再拆 ROS package；不在接口仍变化时只为目录好看而拆包。
- 将验收入口按 `voice/`、`offline/`、`control/`、`navigation/` 分类，根脚本只做稳定命令路由；
  用 manifest 驱动帮助文本和门禁，逐步删除只包一层命令的重复 smoke 脚本。

## 11. 下一阶段优先级

1. 完成剩余结构化 JSON topic 的 typed/diagnostics 迁移并统一 QoS。
2. 抽取 online/offline 共用 Agent 控制面，缩小两个主节点接口和职责。
3. 整理验收脚本 manifest 与分组目录，保留兼容的单一用户入口。
4. 将 C++ bridge 组件化/Lifecycle 化，补 deadline、liveliness 和故障诊断。
5. 完成以上架构阶段后，再继续 Nav2 演示和用户记忆功能。
