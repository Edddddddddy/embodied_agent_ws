# 项目不足与优化路线

本文档记录当前项目相对“产品级语音机器人系统”的不足。它不是否定当前版本；
当前版本已经适合作为 ROS 2 / C++ 求职展示项目。本文的作用是帮助后续开发保持方向：
优先补稳定性、证据和演示可信度，而不是盲目堆功能。

## 1. 真实语音稳定性仍依赖环境

现状：

- `continuous-offline/online` 已能跑通，但 WSL 麦克风、PulseAudio/WSLg、VAD 阈值仍受环境影响。
- 连续语音脚本已默认 `VAD_PROVIDER=auto`：Silero VAD 依赖可用时优先使用成熟声学 VAD，
  不可用时清晰降级到 energy VAD。
- WebRTC VAD/AEC/NS、openWakeWord/sherpa KWS 仍是 seam 或可选项，不是默认强依赖。

优化：

- 继续把 Silero VAD 依赖安装、模型缓存和真实麦克风证据做成更稳定的默认演示路径。
- 引入声学 KWS 默认方案，优先选择部署成本低、可离线运行的方案。
- 把 `wsl-microphone-preflight`、`audio_frontend_calibration.py` 和 profile 推荐做成更闭环的一键诊断。
- 保存真实演示报告，避免“现场听起来能跑”但缺少可复查证据。

## 2. 离线端侧模型效果证据还不够硬

现状：

- llama.cpp、Sherpa-ONNX、Sherpa-TTS、SummerTTS 的工程接口已经具备。
- LoRA 微调、Q8 指令准确率、tokens/s、动作解析准确率还需要系统 benchmark 支撑。

优化：

- 使用 `training/robot_instruction_eval.jsonl` 固化动作解析评估样例；当前已有 39 条代表集，
  覆盖 ASR 错词、多命令、Nav2 目标点/巡航和安全拒绝。
- 用 `docs/OFFLINE_BENCHMARK_REPORT.md` 记录模型大小、首 token、tokens/s、ASR/TTS realtime factor。
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
- 地图、目标点和场景资产仍比较静态。
- 已开始透传 Nav2 action 级别的失败细节，例如 server unavailable、goal rejected、aborted/canceled、
  error code/message、missed waypoints；但还没有进一步语义化归因到 planner/controller/localization
  等具体 Nav2 子系统。

优化：

- 增加固定 Gazebo world、map、waypoint assets。
- 增加 RViz 展示脚本，突出 TF、map、path、goal。
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

- 使用 `bash scripts/acceptance_test.sh release-gate` 输出统一报告。
- 演示前再跑真实麦克风和 Nav2 evidence。
- 后续可增加录屏、截图、RViz 状态导出。

## 7. C++ 占比还可以提高

现状：

- C++ 已覆盖 ActionGuard、typed bridge、audio frontend、simulation executor、SummerTTS service。
- Python 仍承担连续语音状态机、NLU、memory、monitor 和 provider 编排。

优化：

- 优先把诊断、执行状态追踪、部分队列监控 C++ 化。
- 增加 launch test、rclcpp_action 示例和组件化节点展示。
- 保留 Python 在模型编排层的灵活性，不为“全 C++”牺牲迭代速度。

## 8. 用户记忆和声纹仍偏 seam

现状：

- 已有 speaker identity topic、user memory store、mock/sherpa seam 和记忆命令。
- 真实声纹默认链路、隐私边界、偏好影响动作策略仍较浅。

优化：

- 明确用户画像 schema、导出/清理命令和低置信度保护策略。
- 让“慢一点/快一点/默认角度”等偏好真正影响动作参数。
- 低置信度声纹不写入个人记忆。

## 9. 文档丰富但入口偏多

现状：

- README、学习笔记、验收文档、汇报文档、路线图都比较完整。
- 新读者可能不知道先看哪份，面试时也容易讲散。

优化：

- README 保持主入口和快速跑通。
- `PROJECT_PRESENTATION_15MIN.md` 用于汇报。
- `INTERVIEW_QA.md` 用于追问。
- 深入实现继续放 `LEARNING_NOTES.md`。

## 10. 下一阶段优先级

1. 固定真实演示脚本和 evidence 报告。
2. 强化真实麦克风稳定性，默认接入一个成熟 VAD/KWS 方案。
3. 补离线模型 benchmark 报告和动作准确率评估。
4. 增加 Nav2 map/world/waypoint/RViz 展示资产。
5. 继续整理面试问答和 C++ 技术亮点。
