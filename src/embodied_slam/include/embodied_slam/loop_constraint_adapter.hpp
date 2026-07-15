#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "embodied_slam/pose2d.hpp"

namespace embodied_slam
{

struct LoopConstraintBackendRequest
{
  std::int64_t query_stamp_ns{0};
  std::int64_t candidate_stamp_ns{0};
  Pose2d target_to_source;
};

struct LoopConstraintScanRecord
{
  std::int64_t unique_id{-1};
  std::int64_t stamp_ns{0};
  Pose2d corrected_pose;
  Pose2d sensor_pose;
};

struct ResolvedLoopConstraint
{
  bool available{false};
  LoopConstraintScanRecord query;
  LoopConstraintScanRecord candidate;
  Pose2d query_sensor_pose;
  std::string reason;
};

// 将运行时 descriptor ID 与 Karto 内部 ID 解耦：后端只按原始扫描时间戳解析
// 真正写图的节点，再把 ICP 的 candidate->query 变换转换成 Karto 需要的全局 query pose。
ResolvedLoopConstraint resolveLoopConstraint(
  const LoopConstraintBackendRequest & request,
  const std::vector<LoopConstraintScanRecord> & scans,
  std::int64_t maximum_stamp_delta_ns);

}  // namespace embodied_slam
