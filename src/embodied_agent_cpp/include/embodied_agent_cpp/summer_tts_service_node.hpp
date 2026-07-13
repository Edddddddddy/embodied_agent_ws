#pragma once

#include <memory>
#include <mutex>
#include <list>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "embodied_agent_interfaces/srv/synthesize_speech.hpp"

class SynthesizerTrn;

namespace embodied_agent_cpp
{

class SummerTtsServiceNode : public rclcpp::Node
{
public:
  explicit SummerTtsServiceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SummerTtsServiceNode() override;

private:
  using SynthesizeSpeech = embodied_agent_interfaces::srv::SynthesizeSpeech;

  void load_model();
  void handle_request(
    const std::shared_ptr<SynthesizeSpeech::Request> request,
    std::shared_ptr<SynthesizeSpeech::Response> response);
  std::string make_cache_key(const std::string & text, int32_t speaker_id, float length_scale) const;
  bool try_read_cache(const std::string & key, std::vector<uint8_t> & pcm);
  void write_cache(const std::string & key, const std::vector<uint8_t> & pcm);

  struct CacheEntry
  {
    std::vector<uint8_t> pcm;
    std::list<std::string>::iterator order_it;
  };

  std::string model_path_;
  std::unique_ptr<SynthesizerTrn> synthesizer_;
  float * model_data_{nullptr};
  int32_t model_size_{0};
  int32_t default_speaker_id_{0};
  float default_length_scale_{1.0F};
  bool cache_enabled_{true};
  int32_t cache_max_entries_{64};
  int32_t cache_max_text_chars_{24};
  std::mutex synth_mutex_;
  std::mutex cache_mutex_;
  std::list<std::string> cache_order_;
  std::unordered_map<std::string, CacheEntry> cache_;
  rclcpp::Service<SynthesizeSpeech>::SharedPtr service_;
};

}  // namespace embodied_agent_cpp
