#include <portaudio.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include "embodied_agent_cpp/audio_processing.hpp"

namespace embodied_agent_cpp
{

class AudioFrontendNode : public rclcpp::Node
{
public:
  AudioFrontendNode()
  : Node("audio_frontend"),
    capture_enabled_(declare_parameter("capture_enabled", false)),
    speaker_enabled_(declare_parameter("speaker_enabled", false)),
    microphone_rate_(declare_parameter("microphone_sample_rate", 16000)),
    reference_rate_(declare_parameter("reference_sample_rate", 24000)),
    frame_ms_(declare_parameter("frame_ms", 20)),
    frames_per_buffer_(static_cast<unsigned long>(microphone_rate_ * frame_ms_ / 1000)),
    vad_(declare_parameter("vad_rms_threshold", 0.018)),
    silence_(declare_parameter("silence_timeout_s", 0.4)),
    echo_canceller_(
      microphone_rate_,
      reference_rate_,
      static_cast<std::size_t>(declare_parameter("aec_taps", 64)),
      declare_parameter("aec_step", 0.35),
      declare_parameter("aec_delay_ms", 80))
  {
    cleaned_audio_publisher_ = create_publisher<std_msgs::msg::UInt8MultiArray>(
      "/audio/clean_pcm", rclcpp::SensorDataQoS());
    silence_publisher_ = create_publisher<std_msgs::msg::Empty>("/audio/silence_timeout", 10);
    tts_reference_subscription_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/audio/tts_pcm",
      rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::UInt8MultiArray::SharedPtr message) {
        enqueue_playback(message->data);
      });

    if (!capture_enabled_ && !speaker_enabled_) {
      RCLCPP_INFO(get_logger(), "audio frontend ready with capture and speaker disabled");
      return;
    }

    check_portaudio(Pa_Initialize(), "initialize PortAudio");
    portaudio_initialized_ = true;
    running_.store(true);

    try {
      if (capture_enabled_) {
        check_portaudio(
          Pa_OpenDefaultStream(
            &input_stream_,
            1,
            0,
            paInt16,
            static_cast<double>(microphone_rate_),
            frames_per_buffer_,
            &AudioFrontendNode::input_callback,
            this),
          "open microphone");
        check_portaudio(Pa_StartStream(input_stream_), "start microphone");
        processing_thread_ = std::thread(&AudioFrontendNode::processing_loop, this);
      }

      if (speaker_enabled_) {
        check_portaudio(
          Pa_OpenDefaultStream(
            &output_stream_,
            0,
            1,
            paInt16,
            static_cast<double>(reference_rate_),
            paFramesPerBufferUnspecified,
            nullptr,
            nullptr),
          "open speaker");
        check_portaudio(Pa_StartStream(output_stream_), "start speaker");
        playback_thread_ = std::thread(&AudioFrontendNode::playback_loop, this);
      }
    } catch (...) {
      shutdown_audio();
      throw;
    }

    RCLCPP_INFO(
      get_logger(),
      "C++ audio frontend ready: capture=%s speaker=%s frame=%dms",
      capture_enabled_ ? "true" : "false",
      speaker_enabled_ ? "true" : "false",
      frame_ms_);
  }

  ~AudioFrontendNode() override
  {
    shutdown_audio();
  }

private:
  void shutdown_audio()
  {
    if (input_stream_ != nullptr) {
      Pa_StopStream(input_stream_);
    }
    running_.store(false);
    input_condition_.notify_all();
    playback_condition_.notify_all();
    if (processing_thread_.joinable()) {
      processing_thread_.join();
    }
    if (playback_thread_.joinable()) {
      playback_thread_.join();
    }
    if (input_stream_ != nullptr) {
      Pa_CloseStream(input_stream_);
    }
    if (output_stream_ != nullptr) {
      Pa_StopStream(output_stream_);
      Pa_CloseStream(output_stream_);
    }
    if (portaudio_initialized_) {
      Pa_Terminate();
      portaudio_initialized_ = false;
    }
  }
  static int input_callback(
    const void * input,
    void *,
    unsigned long frame_count,
    const PaStreamCallbackTimeInfo *,
    PaStreamCallbackFlags,
    void * user_data)
  {
    auto * node = static_cast<AudioFrontendNode *>(user_data);
    if (input == nullptr || !node->running_.load()) {
      return paContinue;
    }
    const auto * samples = static_cast<const int16_t *>(input);
    std::vector<int16_t> frame(samples, samples + frame_count);
    {
      std::lock_guard<std::mutex> lock(node->input_mutex_);
      if (node->input_queue_.size() >= kMaximumInputFrames) {
        node->input_queue_.pop_front();
        ++node->dropped_input_frames_;
      }
      node->input_queue_.push_back(std::move(frame));
    }
    node->input_condition_.notify_one();
    return paContinue;
  }

  void processing_loop()
  {
    while (true) {
      std::vector<int16_t> frame;
      {
        std::unique_lock<std::mutex> lock(input_mutex_);
        input_condition_.wait(lock, [this] {
          return !running_.load() || !input_queue_.empty();
        });
        if (input_queue_.empty() && !running_.load()) {
          return;
        }
        frame = std::move(input_queue_.front());
        input_queue_.pop_front();
      }

      auto cleaned = echo_canceller_.process(frame);
      std_msgs::msg::UInt8MultiArray message;
      message.data.resize(cleaned.size() * sizeof(int16_t));
      std::memcpy(message.data.data(), cleaned.data(), message.data.size());
      cleaned_audio_publisher_->publish(std::move(message));

      const bool speech = vad_.is_speech(cleaned);
      const double frame_seconds = static_cast<double>(cleaned.size()) / microphone_rate_;
      if (silence_.update(speech, frame_seconds)) {
        silence_publisher_->publish(std_msgs::msg::Empty());
      }
    }
  }

  void enqueue_playback(const std::vector<uint8_t> & bytes)
  {
    if (!speaker_enabled_ || bytes.empty() || bytes.size() % sizeof(int16_t) != 0) {
      return;
    }
    std::vector<int16_t> samples(bytes.size() / sizeof(int16_t));
    std::memcpy(samples.data(), bytes.data(), bytes.size());
    {
      std::lock_guard<std::mutex> lock(playback_mutex_);
      if (playback_queue_.size() >= kMaximumPlaybackChunks) {
        playback_queue_.pop_front();
        ++dropped_playback_chunks_;
      }
      playback_queue_.push_back(std::move(samples));
    }
    playback_condition_.notify_one();
  }

  void playback_loop()
  {
    while (true) {
      std::vector<int16_t> samples;
      {
        std::unique_lock<std::mutex> lock(playback_mutex_);
        playback_condition_.wait(lock, [this] {
          return !running_.load() || !playback_queue_.empty();
        });
        if (playback_queue_.empty() && !running_.load()) {
          return;
        }
        samples = std::move(playback_queue_.front());
        playback_queue_.pop_front();
      }
      echo_canceller_.add_reference(samples);
      const PaError error = Pa_WriteStream(output_stream_, samples.data(), samples.size());
      if (error != paNoError && error != paOutputUnderflowed) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "speaker write failed: %s", Pa_GetErrorText(error));
      }
    }
  }

  static void check_portaudio(PaError error, const std::string & operation)
  {
    if (error != paNoError) {
      throw std::runtime_error(operation + " failed: " + Pa_GetErrorText(error));
    }
  }

  static constexpr std::size_t kMaximumInputFrames = 50;
  static constexpr std::size_t kMaximumPlaybackChunks = 100;

  bool capture_enabled_;
  bool speaker_enabled_;
  int microphone_rate_;
  int reference_rate_;
  int frame_ms_;
  unsigned long frames_per_buffer_;
  EnergyVad vad_;
  SilenceDetector silence_;
  NlmsEchoCanceller echo_canceller_;
  std::atomic<bool> running_{false};
  bool portaudio_initialized_{false};
  PaStream * input_stream_{nullptr};
  PaStream * output_stream_{nullptr};

  std::deque<std::vector<int16_t>> input_queue_;
  std::mutex input_mutex_;
  std::condition_variable input_condition_;
  std::thread processing_thread_;
  std::size_t dropped_input_frames_{0};

  std::deque<std::vector<int16_t>> playback_queue_;
  std::mutex playback_mutex_;
  std::condition_variable playback_condition_;
  std::thread playback_thread_;
  std::size_t dropped_playback_chunks_{0};

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr cleaned_audio_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr silence_publisher_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr tts_reference_subscription_;
};

}  // namespace embodied_agent_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<embodied_agent_cpp::AudioFrontendNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("audio_frontend"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
