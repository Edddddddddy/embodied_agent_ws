# Sherpa-ONNX ZipFormer ASR 真实部署笔记

本文档记录本项目离线 ASR 层的最小真实部署路径。目标是先把
`sherpa-onnx + ZipFormer 流式 ASR + Offline Agent provider seam` 跑通，
不被 llama.cpp、Sherpa-TTS、Gazebo 等更重依赖干扰。

## 1. 为什么新增 ASR-only 入口

完整离线链路需要：

```text
Sherpa-ONNX ASR -> llama.cpp LLM -> Sherpa-TTS -> ROS2 Action -> Gazebo
```

其中 llama.cpp 编译、Qwen GGUF、TTS 模型都比较重。真实部署时如果一上来跑
`scripts/setup_offline_runtime.sh`，失败原因很容易混在一起。因此本阶段新增：

- `scripts/setup_sherpa_asr_runtime.sh`：只安装 `sherpa-onnx` 并下载 ZipFormer ASR 模型。
- `scripts/sherpa_asr_smoke.py`：只加载 ASR provider，对官方 test wav 做一次真实解码。
- `bash scripts/acceptance_test.sh sherpa-asr-preflight`：检查 Python 包和模型文件。
- `bash scripts/acceptance_test.sh sherpa-asr-smoke`：执行真实 ASR 解码。
- `bash scripts/acceptance_test.sh offline-sherpa-typed`：把真实 Sherpa ASR/TTS 接入
  Offline Agent，并验证 typed Action 仿真控制闭环。

这样可以先证明“离线 ASR 推理框架部署成功”，再继续接 llama.cpp 和 TTS。

## 2. 安装 ASR-only 运行时

```bash
cd /home/ubuntu/embodied_agent_ws
source .venv/bin/activate
bash scripts/setup_sherpa_asr_runtime.sh
```

默认模型目录：

```text
models/sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16/
```

关键文件：

```text
tokens.txt
encoder-epoch-99-avg-1.int8.onnx
decoder-epoch-99-avg-1.int8.onnx
joiner-epoch-99-avg-1.int8.onnx
test_wavs/0.wav
```

## 3. 验证命令

只做预检：

```bash
bash scripts/acceptance_test.sh sherpa-asr-preflight
```

真实解码：

```bash
bash scripts/acceptance_test.sh sherpa-asr-smoke
```

完整 ROS2 typed Action 控制链路：

```bash
bash scripts/acceptance_test.sh offline-sherpa-typed
```

预期输出是 JSON，包含：

- `ok: true`
- `text`: 识别文本
- `partial_count` / `final_count`
- `audio_seconds`
- `total_ms`
- `rtf`

其中 `rtf < 1` 表示解码速度快于音频实时长度。

## 4. 关键代码位置

| 模块 | 文件 | 关键函数 |
| --- | --- | --- |
| ASR-only 安装 | `scripts/setup_sherpa_asr_runtime.sh` | `download_file()` |
| ASR 预检/冒烟 | `scripts/sherpa_asr_smoke.py` | `check_model_dir()`、`read_pcm16_wav()`、`run_smoke()` |
| ASR 到 typed Action 链路 | `scripts/smoke_test_offline_sherpa_typed_simulation.sh` | 启动 llama-server、Offline Agent、simulation_control |
| 链路探针 | `tests/integration/control/test_offline_sherpa_typed_simulation.py` | `OfflineSherpaTypedProbe`、`main()` |
| 离线 ASR provider | `src/embodied_offline_agent/embodied_offline_agent/providers/sherpa_asr.py` | `SherpaZipformerAsr.push_audio()`、`commit()` |
| Offline Agent 接入 | `src/embodied_offline_agent/embodied_offline_agent/offline_agent_node.py` | `_create_providers()`、`_run_asr()`、`_on_asr_final()` |

## 5. 与完整离线链路的关系

ASR-only 验收通过后，只能说明：

```text
PCM/WAV -> sherpa-onnx ZipFormer -> transcript
```

如果要验证 Sherpa 模型参与的 ROS2 控制闭环，运行：

```bash
bash scripts/acceptance_test.sh offline-sherpa-typed
```

它会证明：

```text
Sherpa-TTS command audio
  -> Sherpa ZipFormer ASR transcript
  -> Offline Agent action candidate
  -> ActionGuard typed RobotCommand
  -> ExecuteRobotCommand Action result
  -> simulation /cmd_vel
```

如果要验证完整离线 Agent release gate，需要继续运行：

```bash
bash scripts/setup_offline_runtime.sh
bash scripts/acceptance_test.sh offline
```

完整离线链路会额外验证：

- llama.cpp server 是否可用；
- Qwen GGUF 是否可加载；
- Sherpa-TTS 是否可合成；
- Offline Agent 是否能从 ASR final 进入动作解析和 ROS2 控制链路。

## 6. 常见问题

### `sherpa_onnx is not installed`

运行：

```bash
bash scripts/setup_sherpa_asr_runtime.sh
```

### 模型文件缺失或过小

重新执行安装脚本。脚本会检查最小文件大小，已存在且大小合格的文件不会重复下载。

### wav 格式错误

`scripts/sherpa_asr_smoke.py` 要求测试 wav 为：

- mono
- 16kHz
- PCM16

如果使用自己的 wav，需要先转换：

```bash
ffmpeg -i input.wav -ac 1 -ar 16000 -sample_fmt s16 output.wav
python3 scripts/sherpa_asr_smoke.py --wav output.wav
```
