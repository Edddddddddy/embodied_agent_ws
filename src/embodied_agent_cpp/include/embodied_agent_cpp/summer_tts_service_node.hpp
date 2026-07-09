#pragma once

#include <memory>
#include <mutex>
#include <string>
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

  std::string model_path_;
  std::unique_ptr<SynthesizerTrn> synthesizer_;
  float * model_data_{nullptr};
  int32_t model_size_{0};
  int32_t default_speaker_id_{0};
  float default_length_scale_{1.0F};
  std::mutex synth_mutex_;
  rclcpp::Service<SynthesizeSpeech>::SharedPtr service_;
};

}  // namespace embodied_agent_cpp
