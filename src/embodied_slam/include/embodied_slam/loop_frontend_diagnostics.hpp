#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace embodied_slam
{

enum class LoopMatcherEventType
{
  kCoarseCheck,
  kFineCheck,
  kRejected,
  kUnknown,
};

struct LoopMatcherEvent
{
  LoopMatcherEventType type{LoopMatcherEventType::kUnknown};
  std::optional<double> response;
  std::optional<double> response_threshold;
  std::optional<double> variance_x;
  std::optional<double> variance_y;
  std::optional<double> variance_threshold;
  std::string raw;
};

// 将 karto listener 的人类可读字符串转换为稳定字段，避免评估脚本依赖日志文案切片。
LoopMatcherEvent parseLoopMatcherEvent(const std::string & message);
std::string toString(LoopMatcherEventType type);

struct LoopCandidateScan
{
  int state_id{0};
  double x{0.0};
  double y{0.0};
  bool near_linked{false};
  bool current{false};
};

struct LoopCandidateTopology
{
  std::size_t historical_scan_count{0};
  std::size_t geometric_near_count{0};
  std::size_t near_linked_count{0};
  std::size_t eligible_unlinked_count{0};
  std::size_t qualifying_chain_count{0};
  std::size_t maximum_eligible_chain_size{0};
  std::size_t linked_terminated_chain_max{0};
  std::size_t trailing_chain_size{0};
  std::size_t minimum_chain_size{0};
  double search_distance_m{0.0};
  std::string primary_reason;
};

// 复现 FindPossibleLoopClosure 的候选拓扑判定，但不调用 scan matcher、也不修改图。
LoopCandidateTopology summarizeLoopCandidateTopology(
  const std::vector<LoopCandidateScan> & scans,
  double current_x,
  double current_y,
  double search_distance_m,
  std::size_t minimum_chain_size);

}  // namespace embodied_slam
