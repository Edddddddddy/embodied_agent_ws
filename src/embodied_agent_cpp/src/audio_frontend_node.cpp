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

#include "embodied_agent_interfaces/msg/audio_frontend_status.hpp"
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
    vad_provider_(declare_parameter("vad_provider", "energy")),
    endpoint_events_enabled_(declare_parameter("endpoint_events_enabled", true)),
    silence_timeout_s_(declare_parameter("silence_timeout_s", 0.4)),
    speech_end_silence_s_(declare_parameter("speech_end_silence_s", silence_timeout_s_)),
    min_utterance_s_(declare_parameter("min_utterance_ms", 100.0) / 1000.0),
    max_utterance_s_(declare_parameter("max_utterance_s", 12.0)),
    vad_rms_threshold_(declare_parameter("vad_rms_threshold", 0.018)),
    metrics_period_s_(declare_parameter("metrics_period_s", 0.5)),
    vad_(vad_rms_threshold_),
    endpoint_(speech_end_silence_s_, min_utterance_s_, max_utterance_s_),
    audio_enhancer_name_(declare_parameter("audio_enhancer", "nlms")),
    aec_enabled_(declare_parameter("aec_enabled", true)),
    noise_suppression_enabled_(declare_parameter("noise_suppression_enabled", false)),
    auto_gain_enabled_(declare_parameter("auto_gain_enabled", false))
  {
    audio_enhancer_ = create_audio_enhancer();
    cleaned_audio_publisher_ = create_publisher<std_msgs::msg::UInt8MultiArray>(
      "/audio/clean_pcm", rclcpp::SensorDataQoS());
    silence_publisher_ = create_publisher<std_msgs::msg::Empty>("/audio/silence_timeout", 10);
    speech_started_publisher_ = create_publisher<std_msgs::msg::Empty>(
      "/audio/speech_started", 10);
    speech_ended_publisher_ = create_publisher<std_msgs::msg::Empty>(
      "/audio/speech_ended", 10);
    frontend_metrics_publisher_ =
      create_publisher<embodied_agent_interfaces::msg::AudioFrontendStatus>(
      "/audio/frontend_metrics", 10);
    tts_reference_subscription_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/audio/tts_pcm",
      rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::UInt8MultiArray::SharedPtr message) {
        enqueue_playback(message->data);
      });

    if (vad_provider_ == "silero" && !endpoint_events_enabled_) {
      RCLCPP_INFO(
        get_logger(),
        "vad_provider='silero': audio frontend will publish clean PCM only; "
        "external VAD sidecar owns endpoint events");
    } else if (vad_provider_ != "energy") {
      RCLCPP_WARN(
        get_logger(),
        "vad_provider='%s' is not available yet; falling back to energy VAD",
        vad_provider_.c_str());
    }
    if (audio_enhancer_name_ != "nlms") {
      RCLCPP_WARN(
        get_logger(),
        "audio_enhancer='%s' is not available yet; falling back to NLMS",
        audio_enhancer_name_.c_str());
    }
    if (noise_suppression_enabled_ || auto_gain_enabled_) {
      RCLCPP_WARN(
        get_logger(),
        "noise suppression and auto gain require the future WebRTC enhancer; using NLMS AEC only");
    }

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

      auto cleaned = audio_enhancer_->process(frame);
      const auto metrics = compute_audio_frame_metrics(cleaned, vad_rms_threshold_);
      maybe_publish_metrics(metrics);
      std_msgs::msg::UInt8MultiArray message;
      message.data.resize(cleaned.size() * sizeof(int16_t));
      std::memcpy(message.data.data(), cleaned.data(), message.data.size());
      cleaned_audio_publisher_->publish(std::move(message));

      if (endpoint_events_enabled_) {
        const bool speech = metrics.speech;
        const double frame_seconds = static_cast<double>(cleaned.size()) / microphone_rate_;
        const auto endpoint_event = endpoint_.update(speech, frame_seconds);
        if (endpoint_event.speech_started) {
          speech_started_publisher_->publish(std_msgs::msg::Empty());
        }
        if (endpoint_event.speech_ended) {
          speech_ended_publisher_->publish(std_msgs::msg::Empty());
          silence_publisher_->publish(std_msgs::msg::Empty());
        }
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
      audio_enhancer_->add_reference(samples);
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

  std::unique_ptr<AudioEnhancer> create_audio_enhancer()
  {
    AudioEnhancerConfig config;
    config.microphone_rate = microphone_rate_;
    config.reference_rate = reference_rate_;
    config.aec_taps = static_cast<std::size_t>(declare_parameter("aec_taps", 64));
    config.aec_step = declare_parameter("aec_step", 0.35);
    config.aec_delay_ms = declare_parameter("aec_delay_ms", 80);
    config.aec_enabled = aec_enabled_;
    config.noise_suppression_enabled = noise_suppression_enabled_;
    config.auto_gain_enabled = auto_gain_enabled_;
    return std::make_unique<NlmsAudioEnhancer>(config);
  }

  void maybe_publish_metrics(const AudioFrameMetrics & metrics)
  {
    if (metrics_period_s_ <= 0.0 || frontend_metrics_publisher_ == nullptr) {
      return;
    }
    const auto now = get_clock()->now();
    if (last_metrics_publish_.nanoseconds() != 0 &&
      (now - last_metrics_publish_).seconds() < metrics_period_s_)
    {
      return;
    }
    last_metrics_publish_ = now;
    embodied_agent_interfaces::msg::AudioFrontendStatus message;
    message.stamp = now;
    message.rms = static_cast<float>(metrics.rms);
    message.peak = static_cast<uint32_t>(metrics.peak);
    message.speech = metrics.speech;
    message.vad_provider = vad_provider_;
    message.endpoint_events_enabled = endpoint_events_enabled_;
    message.audio_enhancer_requested = audio_enhancer_name_;
    message.audio_enhancer_active = "nlms";
    message.aec_active = aec_enabled_;
    message.noise_suppression_requested = noise_suppression_enabled_;
    message.noise_suppression_active = false;
    message.auto_gain_requested = auto_gain_enabled_;
    message.auto_gain_active = false;
    message.dropped_input_frames = dropped_input_frames_;
    message.dropped_playback_chunks = dropped_playback_chunks_;
    frontend_metrics_publisher_->publish(message);
  }

  static constexpr std::size_t kMaximumInputFrames = 50;
  static constexpr std::size_t kMaximumPlaybackChunks = 100;

  bool capture_enabled_;
  bool speaker_enabled_;
  int microphone_rate_;
  int reference_rate_;
  int frame_ms_;
  unsigned long frames_per_buffer_;
  std::string vad_provider_;
  bool endpoint_events_enabled_;
  double silence_timeout_s_;
  double speech_end_silence_s_;
  double min_utterance_s_;
  double max_utterance_s_;
  double vad_rms_threshold_;
  double metrics_period_s_;
  EnergyVad vad_;
  SpeechEndpointDetector endpoint_;
  std::string audio_enhancer_name_;
  bool aec_enabled_;
  bool noise_suppression_enabled_;
  bool auto_gain_enabled_;
  std::unique_ptr<AudioEnhancer> audio_enhancer_;
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
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr speech_started_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr speech_ended_publisher_;
  rclcpp::Publisher<embodied_agent_interfaces::msg::AudioFrontendStatus>::SharedPtr
    frontend_metrics_publisher_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr tts_reference_subscription_;
  rclcpp::Time last_metrics_publish_{0, 0, RCL_ROS_TIME};
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
