# 项目完成度与验收报告

验收日期：2026-07-01；环境：WSL Ubuntu 24.04、ROS 2 Jazzy、16 vCPU、约 8 GB RAM。

## 总结结论

项目已经达到“可演示、可测试、可继续接真实机器人”的工程原型标准：在线 Agent、真实离线模型、动作安全和 UART/SPI 抽象均已打通，mock 和无麦克风真实语音闭环可以自动验收。

项目尚未达到“最初描述中的所有数字都可作为正式成果声明”的标准。主要未完成项是 LoRA 训练与独立指令集评测、真实声学环境 AEC/P95 延迟测试、实体 UART/SPI/电机联调。未经微调的 Qwen3-0.6B 在 8 条种子集上的模型动作准确率只有 25%，因此不能声称模型保持约 85% 指令遵循率。

## 逐项验收矩阵

| 原始要求 | 状态 | 当前证据与结论 |
|---|---|---|
| 在线实时 ASR | 已实现并实测 | 云 TTS 回灌 ASR，finalization 147–173 ms；短句存在“停止/停职”同音误识别，动作闭环另用文本入口验收 |
| 唤醒词检测 | 部分完成 | 精确唤醒门控有单测；云 ASR 可识别。离线 ZipFormer 对合成短唤醒词会误识别，生产环境仍建议专用 KWS/hotword |
| 回声消除 | 算法完成、声学待验收 | C++ NLMS AEC 和收敛单测通过；WSLg 设备可打开，但未在真实扬声器/麦克风距离下测 ERLE、双讲和延迟 |
| 0.4 秒静音断句 | 已完成 | C++ 单测验证 400 ms 只触发一次；在线和离线 ASR 均消费同一事件 |
| 在线 LLM/TTS 全流式 | 已完成 | LLM HTTP 连接复用，TTS WebSocket 持久连接和 response commit；ROS 真实动作链路通过 |
| 在线 LLM 首 token <1 s | 稳态达标 | 预热后 provider 实测 460–537 ms，ROS 实测 424–528 ms；冷启动实测 1.63–2.34 s，因此节点在发布 ready 前预热 |
| 在线 TTS 首音频 <300 ms | 边界达标 | provider 实测 235–248 ms；ROS 实测 231–301 ms，需至少 100 轮 P50/P95 才能正式宣称稳定达标 |
| ZipFormer/sherpa-onnx 流式 ASR | 已完成并实测 | int8 chunk-32 模型，10.05 s 音频计算 505–527 ms，RTF 0.0502–0.0525，commit 到 final <1 ms |
| 离线 0.4 s 静音、ASR <0.6 s | 已完成 | 真实语音闭环 ASR finalization 0.64 ms；算法断句阈值 0.4 s |
| Qwen3-0.6B + llama.cpp | 已完成 | 官方 Q8_0 GGUF 639,446,688 bytes，多次 llama.cpp CPU decode 为 20.14–30.47 tokens/s，超过 8.6 tokens/s 目标 |
| LoRA 微调 | 未执行 | 按用户要求暂不训练；只提供 LLaMA-Factory 配置和种子数据 |
| 模型指令遵循约 85% | 未达到 | 固定 seed 的 8 条种子集：纯模型 2/8=25%；确定性语义仲裁后工程链路 8/8，但不能替代模型准确率 |
| Q8 压缩至原体积 25% | 表述不成立 | Q8 相对 FP16 通常约为一半；当前只有 604 MiB Q8 文件，没有本地同版本 FP16 基线，不能声明 25% |
| Sherpa-TTS 伪流式 | 已完成 | FP32 Melo-TTS RTF 0.53–0.60；按句合成并切为 80 ms PCM 发布块 |
| 消息/音频双缓冲 | 已完成 | 两个容量为 2 的缓冲、背压、异常 abort；真实语音闭环两项 dropped 均为 0 |
| 离线端到端 <3.5 s | 当前样本达标 | 两次真实 ZipFormer→llama.cpp→TTS→动作验收：首音频 2.11–2.84 s，整轮完成 2.62–3.43 s；仍需多轮 P95 |
| 动作解析与 ROS 控制 | 已完成 | 模型标签解析、语义仲裁、C++ schema/限幅、CRC 帧、watchdog、急停、ACK 全链路通过 |
| UART/SPI 外设驱动 | 接口完成、实体待验收 | UART 使用真实 Linux PTY 测试逐字节发送；SPI spidev 编译通过；当前无 `/dev/ttyUSB*`、`/dev/ttyACM*`、`/dev/spidev*` 实体设备 |

## 真实测量摘要

### 在线 provider

```text
LLM cold first token: 1633–2337 ms
LLM warmed first token: 460–537 ms
TTS connection: 522–668 ms
TTS first audio after connection: 235–248 ms
ASR commit to final: 147–173 ms
```

### 离线单模块

```text
ZipFormer ASR RTF: 0.0502–0.0525
ZipFormer ASR commit-to-final: <1 ms
Melo-TTS RTF: 0.53–0.60
llama.cpp decode: 20.14–30.47 tokens/s
```

### 离线真实语音闭环

```text
ASR transcript: 早日向前走一秒
ASR finalization: <1 ms
LLM first token: 1518–2117 ms
silence/input to first audio: 2111–2838 ms
turn complete: 2623–3434 ms
message/audio buffer dropped: 0 / 0
action: move -> C++ ActionGuard -> HardwareController(mock)
```

测试时关闭了唤醒门控，以隔离验证完整模型链路；离线合成语音把“小智”识别成“早日”，但动作主体正确。真实麦克风场景应单独验收唤醒率和误唤醒率。

最终 `all` 验收中，三个 ROS 包共 33 项单元测试全部通过，随后所有 mock、真实云端、真实离线文字和真实离线语音链路均通过。

## 一键验收

```bash
cd /home/ubuntu/embodied_agent_ws
source scripts/activate.sh

# 单元测试和全部 mock 链路
bash scripts/acceptance_test.sh mock

# 真实云 API，使用极少文本
bash scripts/acceptance_test.sh online

# 真实离线模型、基准、文字和语音闭环
bash scripts/acceptance_test.sh offline

# 全部执行
bash scripts/acceptance_test.sh all
```

若要把“85% 指令遵循”和延迟写成正式成果，下一阶段必须：扩充并冻结独立测试集、执行 LoRA、至少采集 100 轮 P50/P95、接入真实麦克风/扬声器/MCU，并记录原始日志与硬件规格。
