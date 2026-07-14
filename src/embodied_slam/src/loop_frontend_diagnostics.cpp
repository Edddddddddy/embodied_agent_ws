#include "embodied_slam/loop_frontend_diagnostics.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <regex>

namespace embodied_slam
{
namespace
{

std::optional<double> matchNumber(const std::string & text, const std::regex & pattern)
{
  std::smatch match;
  if (!std::regex_search(text, match, pattern) || match.size() < 2) {
    return std::nullopt;
  }
  try {
    return std::stod(match[1].str());
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

}  // namespace

LoopMatcherEvent parseLoopMatcherEvent(const std::string & message)
{
  static const std::regex response_pattern(
    R"(RESPONSE:\s*([-+0-9.eE]+)\s*\(>\s*([-+0-9.eE]+)\))");
  static const std::regex variance_pattern(
    R"(var:\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*\(<\s*([-+0-9.eE]+)\))");

  LoopMatcherEvent result;
  result.raw = message;
  if (message.find("COARSE RESPONSE:") != std::string::npos) {
    result.type = LoopMatcherEventType::kCoarseCheck;
  } else if (message.find("FINE RESPONSE:") != std::string::npos) {
    result.type = LoopMatcherEventType::kFineCheck;
  } else if (message.find("REJECTED!") != std::string::npos) {
    result.type = LoopMatcherEventType::kRejected;
    return result;
  } else {
    return result;
  }

  std::smatch response_match;
  if (std::regex_search(message, response_match, response_pattern) && response_match.size() >= 3) {
    try {
      result.response = std::stod(response_match[1].str());
      result.response_threshold = std::stod(response_match[2].str());
    } catch (const std::exception &) {
      result.response.reset();
      result.response_threshold.reset();
    }
  }
  if (result.type == LoopMatcherEventType::kCoarseCheck) {
    result.variance_x = matchNumber(message, std::regex(
          R"(var:\s*([-+0-9.eE]+))"));
    std::smatch variance_match;
    if (std::regex_search(message, variance_match, variance_pattern) && variance_match.size() >= 4) {
      try {
        result.variance_x = std::stod(variance_match[1].str());
        result.variance_y = std::stod(variance_match[2].str());
        result.variance_threshold = std::stod(variance_match[3].str());
      } catch (const std::exception &) {
        result.variance_x.reset();
        result.variance_y.reset();
        result.variance_threshold.reset();
      }
    }
  }
  return result;
}

std::string toString(const LoopMatcherEventType type)
{
  switch (type) {
    case LoopMatcherEventType::kCoarseCheck:
      return "coarse_check";
    case LoopMatcherEventType::kFineCheck:
      return "fine_check";
    case LoopMatcherEventType::kRejected:
      return "rejected";
    case LoopMatcherEventType::kUnknown:
      return "unknown";
  }
  return "unknown";
}

LoopCandidateTopology summarizeLoopCandidateTopology(
  const std::vector<LoopCandidateScan> & scans,
  const double current_x,
  const double current_y,
  const double search_distance_m,
  const std::size_t minimum_chain_size)
{
  LoopCandidateTopology result;
  result.minimum_chain_size = minimum_chain_size;
  result.search_distance_m = search_distance_m;
  const double maximum_distance_squared = search_distance_m * search_distance_m;
  std::size_t chain_size = 0;

  for (const auto & scan : scans) {
    if (!scan.current) {
      ++result.historical_scan_count;
    }
    const double dx = scan.x - current_x;
    const double dy = scan.y - current_y;
    const bool geometrically_near = dx * dx + dy * dy <= maximum_distance_squared + 1e-9;
    if (geometrically_near && !scan.current) {
      ++result.geometric_near_count;
    }
    if (geometrically_near) {
      if (scan.near_linked) {
        if (!scan.current) {
          ++result.near_linked_count;
        }
        result.linked_terminated_chain_max = std::max(
          result.linked_terminated_chain_max, chain_size);
        chain_size = 0;
      } else {
        if (!scan.current) {
          ++result.eligible_unlinked_count;
        }
        ++chain_size;
        result.maximum_eligible_chain_size = std::max(
          result.maximum_eligible_chain_size, chain_size);
      }
      continue;
    }

    if (chain_size >= minimum_chain_size) {
      ++result.qualifying_chain_count;
    }
    chain_size = 0;
  }
  result.trailing_chain_size = chain_size;

  if (result.qualifying_chain_count > 0) {
    result.primary_reason = "matcher_candidate_available";
  } else if (result.historical_scan_count < minimum_chain_size) {
    result.primary_reason = "insufficient_history";
  } else if (result.geometric_near_count == 0) {
    result.primary_reason = "no_geometric_neighbor";
  } else if (result.eligible_unlinked_count == 0) {
    result.primary_reason = "all_geometric_neighbors_near_linked";
  } else if (result.maximum_eligible_chain_size < minimum_chain_size) {
    result.primary_reason = "eligible_chain_below_minimum";
  } else if (result.linked_terminated_chain_max >= minimum_chain_size) {
    result.primary_reason = "eligible_chain_terminated_by_near_link";
  } else if (result.trailing_chain_size >= minimum_chain_size) {
    result.primary_reason = "eligible_chain_at_end_not_emitted";
  } else {
    result.primary_reason = "candidate_not_emitted";
  }
  return result;
}

}  // namespace embodied_slam
