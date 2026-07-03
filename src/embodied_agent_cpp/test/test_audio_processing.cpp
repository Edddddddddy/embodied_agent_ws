#include <cmath>
#include <cstdint>
#include <numeric>
#include <vector>

#include "gtest/gtest.h"

#include "embodied_agent_cpp/audio_processing.hpp"

namespace
{

double energy(const std::vector<int16_t> & samples)
{
  return std::accumulate(
    samples.begin(), samples.end(), 0.0,
    [](double total, int16_t sample) {
      return total + static_cast<double>(sample) * sample;
    });
}

TEST(EnergyVadTest, DistinguishesSpeechFromSilence)
{
  embodied_agent_cpp::EnergyVad vad(0.01);
  EXPECT_FALSE(vad.is_speech(std::vector<int16_t>(320, 0)));
  EXPECT_TRUE(vad.is_speech(std::vector<int16_t>(320, 1000)));
}

TEST(AudioFrameMetricsTest, ReportsRmsPeakAndSpeechDecision)
{
  const auto silence = embodied_agent_cpp::compute_audio_frame_metrics(
    std::vector<int16_t>(320, 0), 0.01);
  EXPECT_DOUBLE_EQ(silence.rms, 0.0);
  EXPECT_EQ(silence.peak, 0);
  EXPECT_FALSE(silence.speech);

  const auto voice = embodied_agent_cpp::compute_audio_frame_metrics(
    std::vector<int16_t>(320, 1000), 0.01);
  EXPECT_NEAR(voice.rms, 1000.0 / 32768.0, 1e-6);
  EXPECT_EQ(voice.peak, 1000);
  EXPECT_TRUE(voice.speech);
}

TEST(SilenceDetectorTest, EmitsOnceAfterFourHundredMilliseconds)
{
  embodied_agent_cpp::SilenceDetector detector(0.4);
  EXPECT_FALSE(detector.update(true, 0.02));
  for (int frame = 0; frame < 19; ++frame) {
    EXPECT_FALSE(detector.update(false, 0.02));
  }
  EXPECT_TRUE(detector.update(false, 0.02));
  EXPECT_FALSE(detector.update(false, 0.02));
}

TEST(SpeechEndpointDetectorTest, EmitsStartAndEndForCompleteUtterance)
{
  embodied_agent_cpp::SpeechEndpointDetector detector(0.4, 0.1, 10.0);

  auto event = detector.update(true, 0.02);
  EXPECT_TRUE(event.speech_started);
  EXPECT_FALSE(event.speech_ended);

  event = detector.update(true, 0.08);
  EXPECT_FALSE(event.speech_started);
  EXPECT_FALSE(event.speech_ended);

  for (int frame = 0; frame < 19; ++frame) {
    event = detector.update(false, 0.02);
    EXPECT_FALSE(event.speech_ended);
  }
  event = detector.update(false, 0.02);
  EXPECT_TRUE(event.speech_ended);
  EXPECT_EQ(event.end_reason, embodied_agent_cpp::SpeechEndpointReason::kSilence);

  event = detector.update(false, 0.02);
  EXPECT_FALSE(event.speech_started);
  EXPECT_FALSE(event.speech_ended);
}

TEST(SpeechEndpointDetectorTest, DropsShortNoiseBursts)
{
  embodied_agent_cpp::SpeechEndpointDetector detector(0.4, 0.1, 10.0);

  EXPECT_TRUE(detector.update(true, 0.02).speech_started);
  for (int frame = 0; frame < 20; ++frame) {
    const auto event = detector.update(false, 0.02);
    EXPECT_FALSE(event.speech_ended);
  }
}

TEST(SpeechEndpointDetectorTest, ForcesEndAtMaximumUtterance)
{
  embodied_agent_cpp::SpeechEndpointDetector detector(0.4, 0.0, 0.06);

  EXPECT_TRUE(detector.update(true, 0.02).speech_started);
  EXPECT_FALSE(detector.update(true, 0.02).speech_ended);
  const auto event = detector.update(true, 0.02);

  EXPECT_TRUE(event.speech_ended);
  EXPECT_EQ(event.end_reason, embodied_agent_cpp::SpeechEndpointReason::kMaxDuration);
}

TEST(NlmsEchoCancellerTest, PreservesMicrophoneWithoutReference)
{
  embodied_agent_cpp::NlmsEchoCanceller canceller(16000, 16000, 32, 0.2, 0);
  const std::vector<int16_t> microphone{10, -20, 30};
  EXPECT_EQ(canceller.process(microphone), microphone);
}

TEST(NlmsEchoCancellerTest, LearnsRepeatedEcho)
{
  constexpr double kPi = 3.14159265358979323846;
  embodied_agent_cpp::NlmsEchoCanceller canceller(16000, 16000, 32, 0.15, 0);
  std::vector<int16_t> echo(320);
  for (std::size_t index = 0; index < echo.size(); ++index) {
    echo[index] = static_cast<int16_t>(5000.0 * std::sin(2.0 * kPi * index / 40.0));
  }
  const double input_energy = energy(echo);
  std::vector<int16_t> output;
  for (int iteration = 0; iteration < 20; ++iteration) {
    canceller.add_reference(echo);
    output = canceller.process(echo);
  }
  EXPECT_LT(energy(output), input_energy * 0.1);
}

TEST(NlmsAudioEnhancerTest, BypassesMicrophoneWhenAecIsDisabled)
{
  embodied_agent_cpp::AudioEnhancerConfig config;
  config.microphone_rate = 16000;
  config.reference_rate = 16000;
  config.aec_taps = 32;
  config.aec_step = 0.2;
  config.aec_delay_ms = 0;
  config.aec_enabled = false;
  embodied_agent_cpp::NlmsAudioEnhancer enhancer(config);
  const std::vector<int16_t> reference{1000, -1000, 500, -500};
  const std::vector<int16_t> microphone{10, -20, 30, -40};

  enhancer.add_reference(reference);

  EXPECT_EQ(enhancer.process(microphone), microphone);
}

TEST(NlmsAudioEnhancerTest, UsesNlmsEchoCancellerWhenAecIsEnabled)
{
  constexpr double kPi = 3.14159265358979323846;
  embodied_agent_cpp::AudioEnhancerConfig config;
  config.microphone_rate = 16000;
  config.reference_rate = 16000;
  config.aec_taps = 32;
  config.aec_step = 0.15;
  config.aec_delay_ms = 0;
  config.aec_enabled = true;
  embodied_agent_cpp::NlmsAudioEnhancer enhancer(config);
  std::vector<int16_t> echo(320);
  for (std::size_t index = 0; index < echo.size(); ++index) {
    echo[index] = static_cast<int16_t>(5000.0 * std::sin(2.0 * kPi * index / 40.0));
  }
  const double input_energy = energy(echo);
  std::vector<int16_t> output;
  for (int iteration = 0; iteration < 20; ++iteration) {
    enhancer.add_reference(echo);
    output = enhancer.process(echo);
  }

  EXPECT_LT(energy(output), input_energy * 0.1);
}

}  // namespace
