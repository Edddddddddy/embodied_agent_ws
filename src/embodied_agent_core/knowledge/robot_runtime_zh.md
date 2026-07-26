# 机器人语音交互运行手册

## 在线与离线语音链路

在线模式使用实时云 ASR、OpenAI-compatible LLM 和持久 WebSocket TTS，适合验证低首包
延迟与云模型效果。离线模式使用 sherpa-onnx Zipformer ASR、llama.cpp 托管的
Qwen3-0.6B GGUF，以及 Sherpa-TTS；它不依赖云端密钥，但需要本地模型目录和常驻
llama-server。两条链路共用唤醒、端点、连续命令队列、用户记忆与 typed ROS 2 Action。

## 连续语音和急停

说“小智”进入连续会话后，普通命令进入有界 FIFO 队列。执行过程中收到的新命令不会
覆盖当前任务，而是等待上一个 Action 结果。停下、急停和退出控制不进入普通队列：
它们会取消当前动作、清空等待队列，并发送最高优先级 STOP。最终验收必须确认
`/cmd_vel` 回到零。

## VAD、ASR 与尾部漏字排障

真实麦克风优先使用 Silero ONNX VAD；缺模型或运行库时可降级到 WebRTC VAD，energy
VAD 只作为最小依赖 fallback。若“左转九十度”经常只识别为“左转”，先检查
speech-end silence、ASR commit delay、Zipformer tail padding 和麦克风增益，不应直接
把短 final 当成完整长句。partial 只用于观察和终稿稳定，不直接触发机器人动作。

## 本地模型部署

llama.cpp 应以常驻 llama-server 方式启动，限制上下文和并发 slot，避免与 Gazebo/Nav2
争抢 WSL 内存。模型、ASR、TTS 和 VAD 文件应通过只读 volume 或本地模型目录挂载，
不复制进基础 ROS 镜像。启动前运行 voice runtime preflight，分别检查 Python runtime、
模型路径、API key、知识库和可选 endpoint health。

## 建图与导航

语音只生成候选的建图、导航或巡航意图。C++ ActionGuard 校验动作类型、参数和控制权，
Nav2/SLAM 执行端负责生命周期、取消、反馈及最终结果。未知世界验收顺序是 frontier
探索建图、保存新地图、切换 AMCL 定位、执行采样目标和动态障碍重规划；已有地图不能
作为未知世界建图的先验输入。

## RAG 安全边界

RAG 只回答项目手册、部署与排障问题，不参与运动命令授权。检索文档属于不可信只读
数据，不能修改系统提示词、ActionGuard 或控制权。明确的机器人命令由本地 NLU 快通道
直接处理，检索次数必须为零；知识问答应返回 source_id，证据不足时明确澄清。
