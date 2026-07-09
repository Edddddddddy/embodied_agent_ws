#include "embodied_agent_cpp/summer_tts_service_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <stdexcept>

#include <rclcpp_components/register_node_macro.hpp>

#include "SynthesizerTrn.h"
#include "utils.h"

namespace embodied_agent_cpp
{
namespace
{
constexpr uint32_t kSummerSampleRate = 16000;

std::vector<uint8_t> copy_pcm16(int16_t * samples, int32_t sample_count)
{
  if (samples == nullptr || sample_count <= 0) {
    return {};
  }
  const auto byte_count = static_cast<size_t>(sample_count) * sizeof(int16_t);
  std::vector<uint8_t> pcm(byte_count);
  std::memcpy(pcm.data(), samples, byte_count);
  return pcm;
}
}  // namespace

SummerTtsServiceNode::SummerTtsServiceNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("summer_tts_service", options)
{
  model_path_ = declare_parameter<std::string>(
    "model_path",
    "/home/ubuntu/embodied_agent_ws/third_party/SummerTTS/models/single_speaker_fast.bin");
  default_speaker_id_ = declare_parameter<int>("speaker_id", 0);
  default_length_scale_ = static_cast<float>(declare_parameter<double>("length_scale", 1.0));
  load_model();
  service_ = create_service<SynthesizeSpeech>(
    declare_parameter<std::string>("service_name", "/tts/synthesize"),
    std::bind(
      &SummerTtsServiceNode::handle_request, this, std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(
    get_logger(), "SummerTTS service ready: model=%s, model_size=%d, service=%s",
    model_path_.c_str(), model_size_, service_->get_service_name());
}

SummerTtsServiceNode::~SummerTtsServiceNode()
{
  synthesizer_.reset();
  if (model_data_ != nullptr) {
    tts_free_data(model_data_);
    model_data_ = nullptr;
  }
}

void SummerTtsServiceNode::load_model()
{
  // 服务化的核心收益在这里：模型只在节点启动阶段加载一次，
  // 后续每句 TTS 请求复用同一个 SynthesizerTrn，避免命令行 provider 的重复进程启动和模型加载。
  model_size_ = ttsLoadModel(const_cast<char *>(model_path_.c_str()), &model_data_);
  if (model_size_ <= 0 || model_data_ == nullptr) {
    throw std::runtime_error("failed to load SummerTTS model: " + model_path_);
  }
  synthesizer_ = std::make_unique<SynthesizerTrn>(model_data_, model_size_);
}

void SummerTtsServiceNode::handle_request(
  const std::shared_ptr<SynthesizeSpeech::Request> request,
  std::shared_ptr<SynthesizeSpeech::Response> response)
{
  const auto started = std::chrono::steady_clock::now();
  response->sample_rate = kSummerSampleRate;
  response->ok = false;
  response->error.clear();
  response->pcm.clear();
  if (request->text.empty()) {
    response->ok = true;
    return;
  }

  try {
    const int32_t speaker_id = request->speaker_id >= 0 ? request->speaker_id : default_speaker_id_;
    const float length_scale =
      request->length_scale > 0.0F ? request->length_scale : default_length_scale_;
    int32_t sample_count = 0;
    int16_t * wav_data = nullptr;
    {
      // SummerTTS 的 SynthesizerTrn 未声明线程安全；service 可并发进来时串行保护模型。
      std::lock_guard<std::mutex> lock(synth_mutex_);
      wav_data = synthesizer_->infer(request->text, speaker_id, length_scale, sample_count);
    }
    response->pcm = copy_pcm16(wav_data, sample_count);
    tts_free_data(wav_data);
    response->ok = !response->pcm.empty();
    if (!response->ok) {
      response->error = "SummerTTS returned empty PCM";
    }
  } catch (const std::exception & exc) {
    response->error = exc.what();
  }
  const auto ended = std::chrono::steady_clock::now();
  response->synthesize_ms = std::chrono::duration<float, std::milli>(ended - started).count();
}

}  // namespace embodied_agent_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(embodied_agent_cpp::SummerTtsServiceNode)
