#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

namespace embodied_agent_cpp
{

class EnergyVad
{
public:
  // 纯计算接口便于脱离声卡做确定性单测；PortAudio 回调只负责搬运 PCM。
  explicit EnergyVad(double threshold = 0.018);
  bool is_speech(const std::vector<int16_t> & samples) const;

private:
  double threshold_;
};

class SilenceDetector
{
public:
  // 只有“先听到语音，再连续静音 0.4 秒”才提交一次，避免纯静音反复触发 ASR。
  explicit SilenceDetector(double timeout_seconds = 0.4);
  bool update(bool speech, double frame_seconds);
  void reset();

private:
  double timeout_seconds_;
  double silence_seconds_{0.0};
  bool heard_speech_{false};
  bool emitted_{false};
};

enum class SpeechEndpointReason
{
  kNone,
  kSilence,
  kMaxDuration
};

struct SpeechEndpointEvent
{
  bool speech_started{false};
  bool speech_ended{false};
  SpeechEndpointReason end_reason{SpeechEndpointReason::kNone};
};

class SpeechEndpointDetector
{
public:
  // VAD provider 只回答“当前帧是否有人声”；端点检测负责会话边界。
  // 这样后续把 EnergyVad 替换为 Silero/sherpa VAD 时，ROS 事件和 Agent 逻辑不用改。
  SpeechEndpointDetector(
    double end_silence_seconds = 0.4,
    double min_utterance_seconds = 0.1,
    double max_utterance_seconds = 12.0);

  SpeechEndpointEvent update(bool speech, double frame_seconds);
  void reset();

private:
  SpeechEndpointEvent finish(SpeechEndpointReason reason);

  double end_silence_seconds_;
  double min_utterance_seconds_;
  double max_utterance_seconds_;
  double speech_seconds_{0.0};
  double silence_seconds_{0.0};
  bool in_utterance_{false};
};

class NlmsEchoCanceller
{
public:
  // reference 是扬声器播放流，process 输入麦克风流；内部互斥只保护短时 DSP 状态。
  // 网络请求、日志和模型推理严禁进入音频回调路径。
  NlmsEchoCanceller(
    int microphone_rate = 16000,
    int reference_rate = 24000,
    std::size_t taps = 64,
    double step = 0.35,
    int delay_ms = 80);

  void add_reference(const std::vector<int16_t> & samples);
  std::vector<int16_t> process(const std::vector<int16_t> & microphone_samples);
  void reset();

private:
  std::vector<int16_t> resample_reference(const std::vector<int16_t> & samples) const;

  int microphone_rate_;
  int reference_rate_;
  std::size_t taps_;
  double step_;
  std::size_t delay_samples_;
  std::vector<double> weights_;
  std::deque<double> history_;
  std::deque<int16_t> reference_buffer_;
  bool delay_inserted_{false};
  std::mutex mutex_;
};

}  // namespace embodied_agent_cpp
