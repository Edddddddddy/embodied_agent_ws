#include "embodied_slam/loop_constraint_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>

namespace embodied_slam
{
namespace
{

bool finitePose(const Pose2d & pose)
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
}

Pose2d compose(const Pose2d & left, const Pose2d & right)
{
  const double cosine = std::cos(left.yaw);
  const double sine = std::sin(left.yaw);
  return {
    cosine * right.x - sine * right.y + left.x,
    sine * right.x + cosine * right.y + left.y,
    normalize_angle(left.yaw + right.yaw)};
}

Pose2d inverse(const Pose2d & pose)
{
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  return {
    -cosine * pose.x - sine * pose.y,
    sine * pose.x - cosine * pose.y,
    normalize_angle(-pose.yaw)};
}

const LoopConstraintScanRecord * nearest(
  const std::vector<LoopConstraintScanRecord> & scans,
  const std::int64_t stamp_ns,
  const std::int64_t maximum_delta_ns)
{
  const LoopConstraintScanRecord * best = nullptr;
  std::int64_t best_delta = std::numeric_limits<std::int64_t>::max();
  for (const auto & scan : scans) {
    const auto delta = std::llabs(scan.stamp_ns - stamp_ns);
    if (delta < best_delta ||
      (delta == best_delta && best != nullptr && scan.unique_id < best->unique_id))
    {
      best = &scan;
      best_delta = delta;
    }
  }
  return best != nullptr && best_delta <= maximum_delta_ns ? best : nullptr;
}

}  // namespace

ResolvedLoopConstraint resolveLoopConstraint(
  const LoopConstraintBackendRequest & request,
  const std::vector<LoopConstraintScanRecord> & scans,
  const std::int64_t maximum_stamp_delta_ns)
{
  if (maximum_stamp_delta_ns <= 0 || request.query_stamp_ns <= 0 ||
    request.candidate_stamp_ns <= 0 ||
    request.candidate_stamp_ns >= request.query_stamp_ns ||
    !finitePose(request.target_to_source))
  {
    throw std::invalid_argument("invalid loop constraint backend request");
  }
  ResolvedLoopConstraint output;
  const auto * query = nearest(scans, request.query_stamp_ns, maximum_stamp_delta_ns);
  if (query == nullptr) {
    output.reason = "query_scan_not_resolved";
    return output;
  }
  const auto * candidate = nearest(scans, request.candidate_stamp_ns, maximum_stamp_delta_ns);
  if (candidate == nullptr) {
    output.reason = "candidate_scan_not_resolved";
    return output;
  }
  if (query->unique_id == candidate->unique_id) {
    output.reason = "resolved_to_same_scan";
    return output;
  }
  if (candidate->stamp_ns >= query->stamp_ns) {
    output.reason = "resolved_time_order_invalid";
    return output;
  }
  if (!finitePose(query->corrected_pose) || !finitePose(query->sensor_pose) ||
    !finitePose(candidate->corrected_pose) || !finitePose(candidate->sensor_pose))
  {
    output.reason = "resolved_scan_pose_invalid";
    return output;
  }
  output.available = true;
  output.query = *query;
  output.candidate = *candidate;
  // ICP 输出 T_query_candidate（把 candidate 点变到 query 坐标系）；Karto 的
  // rMean 是 query 传感器在全局系中的姿态，因此必须先取逆再左乘 candidate。
  output.query_sensor_pose = compose(candidate->sensor_pose, inverse(request.target_to_source));
  output.reason = "resolved";
  return output;
}

}  // namespace embodied_slam
