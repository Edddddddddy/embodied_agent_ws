#include "embodied_agent_cpp/summer_tts_service_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <sstream>
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

size_t utf8_codepoint_count(const std::string & text)
{
  // cache_max_text_chars 面向中文短反馈调参，不能直接用 UTF-8 字节数；
  // 这里按非 continuation byte 统计 code point，足够覆盖“好的/正在执行”等演示语。
  return static_cast<size_t>(std::count_if(text.begin(), text.end(), [](unsigned char ch) {
    return (ch & 0xC0U) != 0x80U;
  }));
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
  cache_enabled_ = declare_parameter<bool>("cache_enabled", true);
  cache_max_entries_ = declare_parameter<int>("cache_max_entries", 64);
  cache_max_text_chars_ = declare_parameter<int>("cache_max_text_chars", 24);
  load_model();
  service_ = create_service<SynthesizeSpeech>(
    declare_parameter<std::string>("service_name", "/tts/synthesize"),
    std::bind(
      &SummerTtsServiceNode::handle_request, this, std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(
    get_logger(),
    "SummerTTS service ready: model=%s, model_size=%d, service=%s, cache=%s/%d entries, max_text=%d",
    model_path_.c_str(), model_size_, service_->get_service_name(),
    cache_enabled_ ? "on" : "off", cache_max_entries_, cache_max_text_chars_);
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

std::string SummerTtsServiceNode::make_cache_key(
  const std::string & text, int32_t speaker_id, float length_scale) const
{
  // 缓存必须把 speaker 和 length_scale 放进 key：同一句话在不同说话人/语速下 PCM 不同，
  // 如果只按 text 命中会把错误音色复用给后续请求。
  std::ostringstream oss;
  oss << speaker_id << '\n' << std::fixed << std::setprecision(3) << length_scale << '\n' << text;
  return oss.str();
}

bool SummerTtsServiceNode::try_read_cache(const std::string & key, std::vector<uint8_t> & pcm)
{
  if (!cache_enabled_ || cache_max_entries_ <= 0) {
    return false;
  }

  std::lock_guard<std::mutex> lock(cache_mutex_);
  const auto iter = cache_.find(key);
  if (iter == cache_.end()) {
    return false;
  }

  // 命中后提升到队头，保留最常用的短反馈（“收到/好的/正在执行”）。
  cache_order_.splice(cache_order_.begin(), cache_order_, iter->second.order_it);
  iter->second.order_it = cache_order_.begin();
  pcm = iter->second.pcm;
  return true;
}

void SummerTtsServiceNode::write_cache(const std::string & key, const std::vector<uint8_t> & pcm)
{
  if (!cache_enabled_ || cache_max_entries_ <= 0 || pcm.empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(cache_mutex_);
  const auto existing = cache_.find(key);
  if (existing != cache_.end()) {
    existing->second.pcm = pcm;
    cache_order_.splice(cache_order_.begin(), cache_order_, existing->second.order_it);
    existing->second.order_it = cache_order_.begin();
    return;
  }

  cache_order_.push_front(key);
  cache_.emplace(key, CacheEntry{pcm, cache_order_.begin()});
  while (static_cast<int32_t>(cache_.size()) > cache_max_entries_) {
    const auto & evicted_key = cache_order_.back();
    cache_.erase(evicted_key);
    cache_order_.pop_back();
  }
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
  response->cache_hit = false;
  if (request->text.empty()) {
    response->ok = true;
    return;
  }

  try {
    const int32_t speaker_id = request->speaker_id >= 0 ? request->speaker_id : default_speaker_id_;
    const float length_scale =
      request->length_scale > 0.0F ? request->length_scale : default_length_scale_;
    const bool cacheable = cache_enabled_ && cache_max_entries_ > 0 &&
      static_cast<int32_t>(utf8_codepoint_count(request->text)) <= cache_max_text_chars_;
    const std::string cache_key =
      cacheable ? make_cache_key(request->text, speaker_id, length_scale) : std::string();
    if (cacheable && try_read_cache(cache_key, response->pcm)) {
      response->ok = true;
      response->cache_hit = true;
      const auto ended = std::chrono::steady_clock::now();
      response->synthesize_ms = std::chrono::duration<float, std::milli>(ended - started).count();
      return;
    }

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
    if (response->ok && cacheable) {
      // 只缓存短反馈文本，避免长句占用内存；完整语音仍走实时合成。
      write_cache(cache_key, response->pcm);
    }
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
