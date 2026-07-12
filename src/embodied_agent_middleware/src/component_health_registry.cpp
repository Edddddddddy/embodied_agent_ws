#include "embodied_agent_middleware/component_health_registry.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <utility>

namespace embodied_agent_middleware
{

void ComponentHealthRegistry::update(ComponentHealthSample sample)
{
  if (sample.component.empty()) {
    return;
  }
  samples_[sample.component] = std::move(sample);
}

SystemReadinessSnapshot ComponentHealthRegistry::evaluate(
  const std::vector<std::string> & required_components,
  const double now_s, const double stale_timeout_s) const
{
  SystemReadinessSnapshot output;
  output.required_components = required_components;
  const double safe_timeout = std::max(0.0, stale_timeout_s);
  for (const auto & component : required_components) {
    const auto found = samples_.find(component);
    if (found == samples_.end() ||
      now_s - found->second.observed_at_s > safe_timeout)
    {
      output.missing_components.push_back(component);
      continue;
    }
    if (found->second.state == ComponentState::kReady) {
      output.ready_components.push_back(component);
    } else if (found->second.state == ComponentState::kDegraded) {
      output.degraded_components.push_back(component);
    } else {
      output.missing_components.push_back(component);
    }
  }
  output.ready = !required_components.empty() &&
    output.ready_components.size() == required_components.size();
  if (output.ready) {
    output.detail = "all_required_components_ready";
  } else if (!output.degraded_components.empty()) {
    output.detail = "required_component_degraded";
  } else {
    output.detail = "required_component_missing_or_stale";
  }
  return output;
}

void ComponentHealthRegistry::clear()
{
  samples_.clear();
}

std::vector<std::string> parse_component_csv(const std::string & text)
{
  std::vector<std::string> output;
  std::istringstream stream(text);
  std::string token;
  while (std::getline(stream, token, ',')) {
    token.erase(token.begin(), std::find_if(
        token.begin(), token.end(), [](const unsigned char ch) {return !std::isspace(ch);}));
    token.erase(std::find_if(
        token.rbegin(), token.rend(), [](const unsigned char ch) {return !std::isspace(ch);})
      .base(), token.end());
    if (!token.empty() &&
      std::find(output.begin(), output.end(), token) == output.end())
    {
      output.push_back(token);
    }
  }
  return output;
}

}  // namespace embodied_agent_middleware
