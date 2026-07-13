#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace embodied_agent_middleware
{

enum class ComponentState : std::uint8_t
{
  kUnknown = 0,
  kStarting = 1,
  kReady = 2,
  kDegraded = 3,
  kError = 4,
  kStopped = 5,
};

struct ComponentHealthSample
{
  std::string component;
  ComponentState state{ComponentState::kUnknown};
  std::string detail;
  double observed_at_s{0.0};
};

struct SystemReadinessSnapshot
{
  bool ready{false};
  std::vector<std::string> required_components;
  std::vector<std::string> ready_components;
  std::vector<std::string> missing_components;
  std::vector<std::string> degraded_components;
  std::string detail;
};

class ComponentHealthRegistry
{
public:
  void update(ComponentHealthSample sample);
  SystemReadinessSnapshot evaluate(
    const std::vector<std::string> & required_components,
    double now_s, double stale_timeout_s) const;
  void clear();

private:
  std::unordered_map<std::string, ComponentHealthSample> samples_;
};

std::vector<std::string> parse_component_csv(const std::string & text);

}  // namespace embodied_agent_middleware
