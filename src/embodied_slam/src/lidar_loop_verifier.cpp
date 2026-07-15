#include "embodied_slam/lidar_loop_verifier.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace embodied_slam
{

LiveLidarLoopVerifier::LiveLidarLoopVerifier(const LiveLidarLoopVerifierConfig & config)
: config_(config)
{
  if (config_.maximum_cached_scans == 0U) {
    throw std::invalid_argument("maximum cached scans must be positive");
  }
}

void LiveLidarLoopVerifier::cacheScan(
  const std::int64_t stamp_ns, std::vector<LidarPoint2D> points)
{
  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      throw std::invalid_argument("cached scan points must be finite");
    }
  }

  const auto existing = scans_.find(stamp_ns);
  if (existing != scans_.end()) {
    // 同一时间戳可能由 bag 重播重复送达；更新几何但不重复占用淘汰队列。
    existing->second.points = std::move(points);
    return;
  }
  scans_.emplace(stamp_ns, CachedScan{std::move(points)});
  insertion_order_.push_back(stamp_ns);
  while (scans_.size() > config_.maximum_cached_scans) {
    scans_.erase(insertion_order_.front());
    insertion_order_.pop_front();
  }
}

bool LiveLidarLoopVerifier::hasScan(const std::int64_t stamp_ns) const
{
  return scans_.find(stamp_ns) != scans_.end();
}

std::size_t LiveLidarLoopVerifier::cachedScans() const
{
  return scans_.size();
}

std::vector<LiveLidarLoopVerification> LiveLidarLoopVerifier::verify(
  const std::int64_t query_stamp_ns,
  const std::vector<LiveLidarLoopCandidateInput> & candidates) const
{
  std::vector<LiveLidarLoopVerification> output;
  output.reserve(candidates.size());
  const auto query = scans_.find(query_stamp_ns);
  for (const auto & candidate : candidates) {
    LiveLidarLoopVerification verification;
    verification.candidate = candidate;
    if (query == scans_.end()) {
      verification.match.rejection_reason = "query_scan_not_cached";
    } else {
      const auto target = scans_.find(candidate.candidate_stamp_ns);
      if (target == scans_.end()) {
        verification.match.rejection_reason = "candidate_scan_not_cached";
      } else {
        verification.match = matchLidarScans(
          query->second.points, target->second.points,
          candidate.yaw_offset_rad, config_.matcher);
      }
    }
    output.push_back(std::move(verification));
  }
  return output;
}

}  // namespace embodied_slam
