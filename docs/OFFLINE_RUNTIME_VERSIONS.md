# 离线运行时版本收口

本文档记录当前阶段固定的离线推理/语音运行时版本。`third_party/` 与 `models/`
目录不进入 Git 仓库；Git 中只保存可复现部署脚本、版本 pin 和验收入口。

## 当前固定版本

| 组件 | 来源 | 固定版本 | 用途 |
| --- | --- | --- | --- |
| `llama.cpp` | `https://github.com/ggml-org/llama.cpp.git` | `0eca4d490e591d4e93058d07540cf47278a72577` | 本地 Qwen GGUF 推理，提供 OpenAI-compatible streaming API |
| `SummerTTS` | `https://github.com/huakunyang/SummerTTS.git` | `c90e0e8d31e09c98199ab9b5a605af74c179f811` | C++ 独立编译离线 TTS 后端 |
| `sherpa-onnx` | PyPI / k2-fsa | `1.13.3` | ZipFormer ASR、Sherpa-TTS / speaker seam 的 Python runtime |
| `Silero VAD` | `snakers4/silero-vad` | `v6.2.1`, SHA256 `1a153a22...d8788e3` | 纯 ONNX 连续语音端点检测 |
| `onnxruntime` | PyPI | `1.27.0` | Silero VAD CPU 推理，不依赖 PyTorch |
| `LLaMA-Factory` | `https://github.com/hiyouga/LLaMA-Factory.git` | `ea31c43d806162a7fd98065abfef2d974fff5766` | 可选 Qwen3-0.6B LoRA SFT 与 adapter 合并 |

## 部署入口

完整离线运行时：

```bash
bash scripts/setup_offline_runtime.sh
```

SummerTTS 单独部署：

```bash
bash scripts/setup_summer_tts_runtime.sh
```

LoRA 工具链使用独立 `.venv-lora`，不污染 ROS/Agent 主虚拟环境：

```bash
bash scripts/setup_lora_toolchain.sh --dry-run
bash scripts/setup_lora_toolchain.sh
```

如果确实需要临时切换版本，可以通过环境变量覆盖：

```bash
LLAMA_CPP_REF=<commit> bash scripts/setup_offline_runtime.sh
SUMMER_TTS_REF=<commit> bash scripts/setup_summer_tts_runtime.sh
SHERPA_ONNX_VERSION=<version> bash scripts/setup_offline_runtime.sh
```

覆盖版本只建议用于实验分支；阶段性交付版本应更新本文档、脚本默认值和验收记录。

## 验收入口

版本一致性检查：

```bash
bash scripts/acceptance_test.sh offline-runtime-versions
```

分层功能检查：

```bash
bash scripts/acceptance_test.sh llama-cpp-preflight
bash scripts/acceptance_test.sh llama-cpp-smoke
bash scripts/acceptance_test.sh summer-tts-preflight
bash scripts/acceptance_test.sh summer-tts-smoke
bash scripts/acceptance_test.sh summer-pseudo-tts
bash scripts/acceptance_test.sh sherpa-asr-preflight
bash scripts/acceptance_test.sh silero-vad-runtime
bash scripts/acceptance_test.sh lora-q8-pipeline
bash scripts/acceptance_test.sh lora-q8-comparison
```

## 设计取舍

- 固定第三方源码 commit，而不是永远跟随 `main`，避免“昨天能编译、今天不能编译”的演示风险。
- `third_party/` 仍保持 `.gitignore`，避免把大型第三方源码和模型提交进本项目。
- LoRA dry-run 通过只表示配置和命令完整；审计还会校验训练集 manifest、adapter、合并模型、
  F16/Q8 GGUF 和独立对照中的模型/数据/提示词哈希。全部匹配才标记
  `training_quantization_and_holdout_verified`。
- SummerTTS 在 Ubuntu 24.04 / GCC 13 上需要补 `<cstdint>` include；该兼容补丁在
  `scripts/setup_summer_tts_runtime.sh` 中自动执行，只作用于本地 ignored third_party 源码。
- 当前 SummerTTS provider 使用命令行二进制封装，优点是接入快、边界清晰；缺点是每句会重新加载模型。
  后续低延迟优化方向是把 `include/SynthesizerTrn.h` 封装成常驻 C++ ROS 组件。
