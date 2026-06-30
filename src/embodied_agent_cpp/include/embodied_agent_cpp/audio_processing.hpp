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
  explicit EnergyVad(double threshold = 0.018);
  bool is_speech(const std::vector<int16_t> & samples) const;

private:
  double threshold_;
};

class SilenceDetector
{
public:
  explicit SilenceDetector(double timeout_seconds = 0.4);
  bool update(bool speech, double frame_seconds);
  void reset();

private:
  double timeout_seconds_;
  double silence_seconds_{0.0};
  bool heard_speech_{false};
  bool emitted_{false};
};

class NlmsEchoCanceller
{
public:
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

