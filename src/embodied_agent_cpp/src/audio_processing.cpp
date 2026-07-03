#include "embodied_agent_cpp/audio_processing.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace embodied_agent_cpp
{

AudioFrameMetrics compute_audio_frame_metrics(
  const std::vector<int16_t> & samples,
  double speech_threshold)
{
  AudioFrameMetrics metrics;
  if (samples.empty()) {
    return metrics;
  }
  double squared_sum = 0.0;
  int peak = 0;
  for (const int16_t sample : samples) {
    const int magnitude = std::abs(static_cast<int>(sample));
    peak = std::max(peak, magnitude);
    const double normalized = static_cast<double>(sample) / 32768.0;
    squared_sum += normalized * normalized;
  }
  metrics.rms = std::sqrt(squared_sum / static_cast<double>(samples.size()));
  metrics.peak = static_cast<int16_t>(std::min(peak, 32767));
  metrics.speech = metrics.rms >= speech_threshold;
  return metrics;
}

EnergyVad::EnergyVad(double threshold)
: threshold_(threshold)
{
  if (threshold_ < 0.0 || threshold_ > 1.0) {
    throw std::invalid_argument("VAD threshold must be in [0, 1]");
  }
}

bool EnergyVad::is_speech(const std::vector<int16_t> & samples) const
{
  if (samples.empty()) {
    return false;
  }
  const double squared_sum = std::accumulate(
    samples.begin(), samples.end(), 0.0,
    [](double total, int16_t sample) {
      const double normalized = static_cast<double>(sample) / 32768.0;
      return total + normalized * normalized;
    });
  const double rms = std::sqrt(squared_sum / static_cast<double>(samples.size()));
  return rms >= threshold_;
}

SilenceDetector::SilenceDetector(double timeout_seconds)
: timeout_seconds_(timeout_seconds)
{
  if (timeout_seconds_ <= 0.0) {
    throw std::invalid_argument("silence timeout must be positive");
  }
}

bool SilenceDetector::update(bool speech, double frame_seconds)
{
  if (speech) {
    heard_speech_ = true;
    emitted_ = false;
    silence_seconds_ = 0.0;
    return false;
  }
  if (!heard_speech_ || emitted_) {
    return false;
  }
  silence_seconds_ += frame_seconds;
  if (silence_seconds_ + 1e-9 >= timeout_seconds_) {
    emitted_ = true;
    heard_speech_ = false;
    return true;
  }
  return false;
}

void SilenceDetector::reset()
{
  silence_seconds_ = 0.0;
  heard_speech_ = false;
  emitted_ = false;
}

SpeechEndpointDetector::SpeechEndpointDetector(
  double end_silence_seconds,
  double min_utterance_seconds,
  double max_utterance_seconds)
: end_silence_seconds_(end_silence_seconds),
  min_utterance_seconds_(min_utterance_seconds),
  max_utterance_seconds_(max_utterance_seconds)
{
  if (end_silence_seconds_ <= 0.0) {
    throw std::invalid_argument("speech end silence must be positive");
  }
  if (min_utterance_seconds_ < 0.0) {
    throw std::invalid_argument("minimum utterance duration must be non-negative");
  }
  if (max_utterance_seconds_ <= 0.0) {
    throw std::invalid_argument("maximum utterance duration must be positive");
  }
}

SpeechEndpointEvent SpeechEndpointDetector::update(bool speech, double frame_seconds)
{
  if (frame_seconds <= 0.0) {
    return {};
  }

  SpeechEndpointEvent event;
  if (speech) {
    if (!in_utterance_) {
      in_utterance_ = true;
      event.speech_started = true;
    }
    speech_seconds_ += frame_seconds;
    silence_seconds_ = 0.0;
    if (speech_seconds_ + 1e-9 >= max_utterance_seconds_) {
      const auto finished = finish(SpeechEndpointReason::kMaxDuration);
      event.speech_ended = finished.speech_ended;
      event.end_reason = finished.end_reason;
    }
    return event;
  }

  if (!in_utterance_) {
    return event;
  }

  silence_seconds_ += frame_seconds;
  if (silence_seconds_ + 1e-9 < end_silence_seconds_) {
    return event;
  }
  return finish(SpeechEndpointReason::kSilence);
}

void SpeechEndpointDetector::reset()
{
  speech_seconds_ = 0.0;
  silence_seconds_ = 0.0;
  in_utterance_ = false;
}

SpeechEndpointEvent SpeechEndpointDetector::finish(SpeechEndpointReason reason)
{
  const bool long_enough = speech_seconds_ + 1e-9 >= min_utterance_seconds_;
  reset();
  if (!long_enough) {
    return {};
  }
  return SpeechEndpointEvent{false, true, reason};
}

NlmsEchoCanceller::NlmsEchoCanceller(
  int microphone_rate,
  int reference_rate,
  std::size_t taps,
  double step,
  int delay_ms)
: microphone_rate_(microphone_rate),
  reference_rate_(reference_rate),
  taps_(std::max<std::size_t>(8, taps)),
  step_(step),
  delay_samples_(static_cast<std::size_t>(microphone_rate * std::max(0, delay_ms) / 1000)),
  weights_(taps_, 0.0),
  history_(taps_, 0.0)
{
  if (microphone_rate_ <= 0 || reference_rate_ <= 0) {
    throw std::invalid_argument("sample rates must be positive");
  }
  if (step_ <= 0.0 || step_ > 2.0) {
    throw std::invalid_argument("NLMS step must be in (0, 2]");
  }
}

void NlmsEchoCanceller::add_reference(const std::vector<int16_t> & samples)
{
  if (samples.empty()) {
    return;
  }
  const auto resampled = resample_reference(samples);
  std::lock_guard<std::mutex> lock(mutex_);
  if (!delay_inserted_) {
    reference_buffer_.insert(reference_buffer_.end(), delay_samples_, 0);
    delay_inserted_ = true;
  }
  reference_buffer_.insert(reference_buffer_.end(), resampled.begin(), resampled.end());
  const std::size_t maximum = static_cast<std::size_t>(microphone_rate_) * 2;
  while (reference_buffer_.size() > maximum) {
    reference_buffer_.pop_front();
  }
}

std::vector<int16_t> NlmsEchoCanceller::process(
  const std::vector<int16_t> & microphone_samples)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (reference_buffer_.size() < microphone_samples.size()) {
    return microphone_samples;
  }

  std::vector<int16_t> output;
  output.reserve(microphone_samples.size());
  for (const int16_t microphone_sample : microphone_samples) {
    const double reference = static_cast<double>(reference_buffer_.front()) / 32768.0;
    reference_buffer_.pop_front();
    history_.pop_back();
    history_.push_front(reference);

    double predicted = 0.0;
    double energy = 1e-6;
    for (std::size_t index = 0; index < taps_; ++index) {
      predicted += weights_[index] * history_[index];
      energy += history_[index] * history_[index];
    }
    const double desired = static_cast<double>(microphone_sample) / 32768.0;
    const double error = desired - predicted;
    const double scale = step_ * error / energy;
    for (std::size_t index = 0; index < taps_; ++index) {
      weights_[index] += scale * history_[index];
    }
    const auto converted = static_cast<int>(std::lround(error * 32768.0));
    output.push_back(static_cast<int16_t>(std::clamp(converted, -32768, 32767)));
  }
  return output;
}

void NlmsEchoCanceller::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::fill(weights_.begin(), weights_.end(), 0.0);
  std::fill(history_.begin(), history_.end(), 0.0);
  reference_buffer_.clear();
  delay_inserted_ = false;
}

std::vector<int16_t> NlmsEchoCanceller::resample_reference(
  const std::vector<int16_t> & samples) const
{
  if (reference_rate_ == microphone_rate_) {
    return samples;
  }
  const double ratio = static_cast<double>(microphone_rate_) / reference_rate_;
  const auto output_size = static_cast<std::size_t>(
    std::floor(static_cast<double>(samples.size()) * ratio));
  std::vector<int16_t> output;
  output.reserve(output_size);
  for (std::size_t output_index = 0; output_index < output_size; ++output_index) {
    const double source_position = static_cast<double>(output_index) / ratio;
    const auto lower = static_cast<std::size_t>(std::floor(source_position));
    const auto upper = std::min(lower + 1, samples.size() - 1);
    const double fraction = source_position - static_cast<double>(lower);
    const double interpolated =
      (1.0 - fraction) * samples[lower] + fraction * samples[upper];
    output.push_back(static_cast<int16_t>(std::lround(interpolated)));
  }
  return output;
}

NlmsAudioEnhancer::NlmsAudioEnhancer(const AudioEnhancerConfig & config)
: aec_enabled_(config.aec_enabled),
  echo_canceller_(
    config.microphone_rate,
    config.reference_rate,
    config.aec_taps,
    config.aec_step,
    config.aec_delay_ms)
{
}

void NlmsAudioEnhancer::add_reference(const std::vector<int16_t> & samples)
{
  if (!aec_enabled_) {
    return;
  }
  echo_canceller_.add_reference(samples);
}

std::vector<int16_t> NlmsAudioEnhancer::process(
  const std::vector<int16_t> & microphone_samples)
{
  if (!aec_enabled_) {
    return microphone_samples;
  }
  return echo_canceller_.process(microphone_samples);
}

void NlmsAudioEnhancer::reset()
{
  echo_canceller_.reset();
}

}  // namespace embodied_agent_cpp
